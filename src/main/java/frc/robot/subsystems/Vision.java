package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;

// Math imports
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// AprilTag imports
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

// PhotonVision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// PhotonVision simulation imports
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

// CTRE timestamp utility
// CTRE uses a different clock than WPILib. PhotonVision gives us
// FPGA timestamps, but CTRE's addVisionMeasurement needs them
// converted. Think of it like converting time zones.
import com.ctre.phoenix6.Utils;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;


public class Vision extends SubsystemBase {

    // ===== FIELD LAYOUT =====
    // The map of where every AprilTag is on the field.
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // ===== FIELD DIMENSIONS =====
    // Used to reject poses that are "off the field" — clearly wrong.
    // These are the approximate field dimensions in meters.
    private static final double FIELD_LENGTH_METERS = 16.54;
    private static final double FIELD_WIDTH_METERS  = 8.21;
    // We allow a small margin outside the field (robot can overhang)
    private static final double FIELD_MARGIN_METERS = 0.5;

    // ===== SAFEGUARD CONSTANTS =====
    // These are the "filters" that protect against bad vision data.
    // Think of them like bouncers at a club — they decide which
    // measurements are trustworthy enough to let in.

    // If a single tag has ambiguity above this, reject it.
    // Ambiguity ranges from 0 (perfect) to 1 (terrible).
    // The "tag flipping" problem happens with high ambiguity.
    private static final double MAX_AMBIGUITY = 0.2;

    // If the vision pose is this far from our current estimate,
    // reject it. Something is clearly wrong.
    private static final double MAX_POSE_JUMP_METERS = 1.5;

    // If the robot is rotating faster than this (radians/sec),
    // don't trust vision — the image is probably blurry.
    private static final double MAX_ANGULAR_VELOCITY = Math.toRadians(720);

    // ===== STANDARD DEVIATIONS =====
    // Standard deviations control how much we "trust" the vision
    // measurement vs the wheel odometry.
    //
    // Think of it like this:
    //   Small number = "I really trust this measurement"
    //   Big number   = "I'm skeptical of this measurement"
    //
    // These are BASE values that get scaled based on conditions.
    // Format: {x meters, y meters, rotation radians}

    // When we see multiple tags — high confidence
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, Math.toRadians(10));

    // When we see only one tag — lower confidence
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(2.0, 2.0, Math.toRadians(30));


    // ===== CAMERAS =====
    private final PhotonCamera cameraFront = new PhotonCamera("front");
    private final PhotonCamera cameraBack  = new PhotonCamera("back");

    // ===== CAMERA POSITIONS ON ROBOT =====
    // >>> IMPORTANT: Update these once you know mounting positions! <<<

    // Front camera: 0.3m forward, centered, 0.5m up, tilted up 15 deg
    private final Transform3d robotToFrontCamera = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), 0));

    // Back camera: 0.3m backward, centered, 0.5m up, facing backward
    private final Transform3d robotToBackCamera = new Transform3d(
        new Translation3d(-0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180)));

    // ===== POSE ESTIMATORS =====
    // Each camera gets its own PhotonPoseEstimator. This class takes
    // the raw tag detections and calculates "where is the robot?"
    //
    // MULTI_TAG_PNP_ON_COPROCESSOR means:
    //   "If you see multiple tags, use them all together for a better
    //    estimate. Do the math on the coprocessor (Limelight) to save
    //    roboRIO CPU time."
    private final PhotonPoseEstimator frontEstimator;
    private final PhotonPoseEstimator backEstimator;

    // ===== SIMULATION OBJECTS =====
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim backCameraSim;

    // ===== CALLBACKS =====
    // poseSupplier: asks the drivetrain "where do you think we are?"
    // visionConsumer: tells the drivetrain "vision thinks we're HERE"
    private final Supplier<Pose2d> poseSupplier;
    private final VisionMeasurementConsumer visionConsumer;

    // ===== NETWORKTABLES PUBLISHERS =====
    private final BooleanPublisher frontSeesTargetPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/FrontSeesTarget").publish();
    private final BooleanPublisher backSeesTargetPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/BackSeesTarget").publish();
    private final DoublePublisher frontTargetCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/FrontTargetCount").publish();
    private final DoublePublisher backTargetCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/BackTargetCount").publish();
    private final BooleanPublisher frontAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/FrontAccepted").publish();
    private final BooleanPublisher backAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/BackAccepted").publish();
    private final DoublePublisher frontDistancePub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/FrontAvgDistance").publish();
    private final DoublePublisher backDistancePub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/BackAvgDistance").publish();


    // ===== FUNCTIONAL INTERFACE =====
    // This defines the "shape" of the function we'll use to send
    // vision data to the drivetrain. It's like defining a form
    // that says "I need: a pose, a timestamp, and std devs."
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }


    // ===== CONSTRUCTOR =====
    public Vision(
            Supplier<Pose2d> poseSupplier,
            VisionMeasurementConsumer visionConsumer) {
        setName("Vision");
        this.poseSupplier = poseSupplier;
        this.visionConsumer = visionConsumer;

        // Create pose estimators for each camera
        frontEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToFrontCamera);
        // Fallback: if only one tag is visible, use the least-ambiguous pose
        frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        backEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackCamera);
        backEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }


    // ===== SIMULATION SETUP =====
    private void setupSimulation() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 960, Rotation2d.fromDegrees(75));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        frontCameraSim = new PhotonCameraSim(cameraFront, cameraProperties);
        backCameraSim  = new PhotonCameraSim(cameraBack, cameraProperties);

        visionSim.addCamera(frontCameraSim, robotToFrontCamera);
        visionSim.addCamera(backCameraSim, robotToBackCamera);

        frontCameraSim.enableDrawWireframe(true);
        backCameraSim.enableDrawWireframe(true);
    }


    // ===== CORE LOGIC: PROCESS ONE CAMERA =====
    // This is where the magic happens. For each camera, we:
    //   1. Get the pose estimate
    //   2. Run it through all our safeguards
    //   3. If it passes, send it to the drivetrain
    //   4. Publish telemetry so we can debug in AdvantageScope

    private boolean processCamera(
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            BooleanPublisher acceptedPub,
            DoublePublisher distancePub) {

        // Step 1: Ask the estimator for a pose based on what the camera sees
        Optional<EstimatedRobotPose> optionalEstimate = estimator.update(camera.getLatestResult());

        // If the camera didn't see any tags, nothing to do
        if (optionalEstimate.isEmpty()) {
            acceptedPub.set(false);
            distancePub.set(-1);
            return false;
        }

        EstimatedRobotPose estimate = optionalEstimate.get();
        Pose2d estimatedPose = estimate.estimatedPose.toPose2d();
        List<PhotonTrackedTarget> targets = estimate.targetsUsed;
        int tagCount = targets.size();

        // Step 2: Calculate average distance to the tags we see
        double totalDistance = 0;
        for (PhotonTrackedTarget target : targets) {
            totalDistance += target.getBestCameraToTarget()
                .getTranslation().getNorm();
        }
        double avgDistance = totalDistance / tagCount;
        distancePub.set(avgDistance);

        // ===== SAFEGUARD CHECKS =====
        // Each check can reject the measurement.
        // Think of these as a series of security checkpoints.

        // Check 1: Is the pose on the field?
        if (estimatedPose.getX() < -FIELD_MARGIN_METERS
                || estimatedPose.getX() > FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
                || estimatedPose.getY() < -FIELD_MARGIN_METERS
                || estimatedPose.getY() > FIELD_WIDTH_METERS + FIELD_MARGIN_METERS) {
            acceptedPub.set(false);
            return false;
        }

        // Check 2: Single tag with high ambiguity? Reject it.
        // This is the "tag flipping" protection.
        if (tagCount == 1) {
            double ambiguity = targets.get(0).getPoseAmbiguity();
            if (ambiguity > MAX_AMBIGUITY) {
                acceptedPub.set(false);
                return false;
            }
        }

        // Check 3: Is the pose way too far from where we think we are?
        Pose2d currentPose = poseSupplier.get();
        double poseJump = currentPose.getTranslation()
            .getDistance(estimatedPose.getTranslation());
        if (poseJump > MAX_POSE_JUMP_METERS) {
            acceptedPub.set(false);
            return false;
        }

        // ===== CALCULATE TRUST LEVEL =====
        // More tags + closer distance = more trust (smaller std devs)
        // Fewer tags + farther distance = less trust (bigger std devs)

        Matrix<N3, N1> stdDevs;
        if (tagCount >= 2) {
            // Multiple tags: start with high-trust base values
            stdDevs = MULTI_TAG_STD_DEVS;
        } else {
            // Single tag: start with low-trust base values
            stdDevs = SINGLE_TAG_STD_DEVS;
        }

        // Scale trust by distance: farther away = less trust
        // The multiplier grows with the square of distance,
        // because errors compound at range.
        double distanceScale = 1.0 + (avgDistance * avgDistance / 30.0);
        stdDevs = stdDevs.times(distanceScale);

        // ===== SEND TO DRIVETRAIN =====
        // Convert the PhotonVision timestamp (FPGA time) to CTRE time.
        // This is crucial — without this conversion, CTRE will ignore
        // the measurement or apply it at the wrong time.
        double timestamp = Utils.fpgaToCurrentTime(estimate.timestampSeconds);

        visionConsumer.accept(estimatedPose, timestamp, stdDevs);
        acceptedPub.set(true);
        return true;
    }


    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // Process each camera
        processCamera(cameraFront, frontEstimator, frontAcceptedPub, frontDistancePub);
        processCamera(cameraBack, backEstimator, backAcceptedPub, backDistancePub);

        // Publish basic telemetry
        var frontResult = cameraFront.getLatestResult();
        var backResult  = cameraBack.getLatestResult();

        frontSeesTargetPub.set(frontResult.hasTargets());
        backSeesTargetPub.set(backResult.hasTargets());
        frontTargetCountPub.set(frontResult.hasTargets()
            ? frontResult.getTargets().size() : 0);
        backTargetCountPub.set(backResult.hasTargets()
            ? backResult.getTargets().size() : 0);
    }


    // ===== SIMULATION PERIODIC =====
    @Override
    public void simulationPeriodic() {
        visionSim.update(poseSupplier.get());
    }
}