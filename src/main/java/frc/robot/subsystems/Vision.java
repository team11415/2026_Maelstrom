package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;

// Math imports
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// Limelight library — this is the single file you downloaded
import frc.robot.LimelightHelpers;

import java.util.function.Supplier;

/**
 * Vision subsystem using two Limelight 4 cameras for pose estimation.
 *
 * HOW THIS WORKS (the big picture):
 *
 *   The Limelight's own processor detects AprilTags and publishes a robot
 *   pose estimate to NetworkTables. We read that result and fuse it into
 *   the drivetrain's Kalman filter.
 *
 *   Think of it like hiring an accountant (Limelight) instead of doing your
 *   taxes yourself — you just get the final answer.
 *
 *   We use MegaTag2, which fuses:
 *     - What the camera sees (AprilTags)
 *     - What your gyro says (robot heading from the drivetrain)
 *   This eliminates the tag-flipping ambiguity problem and works well
 *   even with only a single AprilTag visible.
 *
 * POSE COMPARISON TELEMETRY:
 *   This subsystem publishes raw per-camera pose data so you can compare
 *   what each camera independently measured vs. what odometry has settled on.
 *
 *   2D Field view topics (drag into AdvantageScope as "Robot" type):
 *     Vision/Poses/CameraA  → Ghost robot showing camera A's raw opinion
 *     Vision/Poses/CameraB  → Ghost robot showing camera B's raw opinion
 *     Vision/Poses/Blended  → Ghost robot showing the final blended pose
 *
 *   Scalar telemetry:
 *     Vision/A_RawPoseX, Y   → Camera A raw position (meters)
 *     Vision/B_RawPoseX, Y   → Camera B raw position (meters)
 *     Vision/OdoPoseX, Y     → Blended odometry position (meters)
 *     Vision/PoseErrorMeters → Distance between camera A and odometry
 *     Vision/PoseLocked      → true when BOTH cameras accepted readings this loop
 *
 * THERMAL MANAGEMENT:
 *   Heat comes from the camera running its vision pipeline. THROTTLE_DISABLED = 200
 *   controls this directly — the pipeline runs roughly once every 6-7 seconds
 *   instead of 30+ times per second when disabled.
 */
public class Vision extends SubsystemBase {

    // ===== LIMELIGHT NAMES =====
    // These must match EXACTLY what you set in each camera's web UI Settings tab.
    // Case-sensitive. The hyphen matters. "limelight_a" would silently fail.
    private static final String LL_A = "limelight-a";
    private static final String LL_B = "limelight-b";

    // ===== NETWORKTABLES HANDLES =====
    // These are our "phone lines" to each Limelight camera.
    // We use them to send commands TO the cameras (like throttle_set).
    private final NetworkTable llATable;
    private final NetworkTable llBTable;

    // ===== THROTTLE CONSTANTS =====
    // throttle_set tells the Limelight how many frames to SKIP between each
    // frame it actually processes for pose estimation.
    //
    //   THROTTLE_ENABLED  = 0   → process every frame (full speed during a match)
    //   THROTTLE_DISABLED = 200 → process 1 frame, skip 200, process 1, skip 200...
    //
    // At 30 fps, throttle=200 means the camera runs its pipeline roughly once
    // every 6-7 seconds while disabled — much cooler.
    public static final int THROTTLE_ENABLED  = 0;
    public static final int THROTTLE_DISABLED = 200;

    // ===== FIELD DIMENSIONS =====
    // Used to reject poses that are clearly impossible ("we're 30 meters away?!")
    private static final double FIELD_LENGTH_METERS = 16.54;
    private static final double FIELD_WIDTH_METERS  = 8.21;
    private static final double FIELD_MARGIN_METERS = 0.5;

    // ===== SAFEGUARD CONSTANTS =====
    // If the robot is spinning faster than this (in rotations per second),
    // don't trust vision — the image is probably blurry from motion.
    // 2.0 RPS = 720 degrees/sec, which is a very fast spin.
    private static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;

    // ===== STANDARD DEVIATIONS =====
    // These control how much the drivetrain "trusts" vision vs. wheel odometry.
    //
    //   Small number = "I really trust this measurement"
    //   Large number = "I'm skeptical of this measurement"
    //
    // Format: {x_meters, y_meters, rotation_radians}
    //
    // Rotation is always 9999999 (essentially infinity) because MegaTag2 uses
    // the gyro for heading — we never want vision to override the gyro's rotation.

    // Two or more tags → higher confidence → tighter uncertainty
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, 9999999);

    // One tag → lower confidence → wider uncertainty
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(1.0, 1.0, 9999999);

    // ===== CALLBACKS =====
    // poseSupplier:   asks the drivetrain "where do you think we are right now?"
    // visionConsumer: tells the drivetrain "vision thinks we're HERE at THIS time"
    // omegaSupplier:  asks the drivetrain "how fast are we spinning?" (radians/sec)
    //                 Divided by (2π) in periodic() to convert to rotations/sec.
    private final Supplier<Pose2d> poseSupplier;
    private final VisionMeasurementConsumer visionConsumer;
    private final java.util.function.DoubleSupplier omegaSupplier;

    // =========================================================================
    // ===== NETWORKTABLES PUBLISHERS ==========================================
    // =========================================================================
    // Think of publishers like the robot's "outbox" — written every 20ms,
    // read by Elastic and AdvantageScope. One-way: robot → dashboard.

    // ---- Per-camera accepted/rejected flag ----
    // true  = reading passed all checks and was sent to the drivetrain this loop
    // false = rejected (camera offline, no tags, spinning too fast, or off-field)
    private final BooleanPublisher aAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/A_Accepted").publish();
    private final BooleanPublisher bAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/B_Accepted").publish();

    // ---- How many AprilTags each camera's MegaTag2 solve used this loop ----
    private final DoublePublisher aTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_TagCount").publish();
    private final DoublePublisher bTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_TagCount").publish();

    // ---- Average distance (meters) to the tags each camera currently sees ----
    private final DoublePublisher aAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_AvgDist").publish();
    private final DoublePublisher bAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_AvgDist").publish();

    // Add this publisher near your other publishers
    private final DoublePublisher sentHeadingPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/SentHeadingDeg").publish();

    // =========================================================================
    // ===== 2D FIELD VIEW PUBLISHERS ==========================================
    // =========================================================================
    // These publish full Pose2d values as [x, y, rotationDegrees] arrays.
    // In AdvantageScope: drag each topic onto the 2D Field view and select
    // "Robot" as the type. Each one becomes a separate colored ghost robot.
    //
    //   Vision/Poses/CameraA  = where camera A alone thinks we are
    //   Vision/Poses/CameraB  = where camera B alone thinks we are
    //   Vision/Poses/Blended  = the final answer (wheels + both cameras fused)
    //
    // CameraA and CameraB only update on ACCEPTED frames — the ghost "freezes"
    // at the last good reading rather than jumping to 0,0 on a reject.

    private final DoubleArrayPublisher aPosePub = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("Vision/Poses/CameraA").publish();

    private final DoubleArrayPublisher bPosePub = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("Vision/Poses/CameraB").publish();

    // NOTE: this is the BLENDED pose publisher — separate from bPosePub (camera B)!
    // Easy to confuse because both start with "b". blendedPosePub = final answer.
    private final DoubleArrayPublisher blendedPosePub = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("Vision/Poses/Blended").publish();

    // =========================================================================
    // ===== SCALAR POSE COMPARISON PUBLISHERS =================================
    // =========================================================================
    // X and Y only (no rotation). Useful for graphing over time in AdvantageScope
    // to watch how the values converge or diverge during a run.

    // Camera A's independent raw pose (X, Y in meters, WPILib field coordinates)
    private final DoublePublisher aRawPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_RawPoseX").publish();
    private final DoublePublisher aRawPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_RawPoseY").publish();

    // Camera B's independent raw pose
    private final DoublePublisher bRawPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_RawPoseX").publish();
    private final DoublePublisher bRawPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_RawPoseY").publish();

    // The drivetrain's blended odometry pose (wheels + all accepted vision updates).
    // This is the position the robot actually uses for aiming and auto paths.
    private final DoublePublisher odoPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/OdoPoseX").publish();
    private final DoublePublisher odoPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/OdoPoseY").publish();

    // Straight-line distance (meters) between camera A's raw reading and odometry.
    // "How much do the camera and wheel-tracking currently disagree?"
    //
    //   < 0.2 m   = great
    //   0.2-0.5 m = ok, normal during fast motion
    //   0.5-1.0 m = investigate (was starting pose reset correctly?)
    //   > 1.0 m   = problem (check pipeline config or field map)
    //
    // Published as -1.0 when camera A has no accepted reading this loop.
    private final DoublePublisher poseErrorPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/PoseErrorMeters").publish();

    // true when BOTH cameras accepted a reading this loop.
    // Both locked = high-confidence pose. One or zero = reduced confidence.
    private final BooleanPublisher poseLockedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/PoseLocked").publish();

    // =========================================================================

    // ===== FUNCTIONAL INTERFACE =====
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    // ===== CONSTRUCTOR =====
    /**
     * Creates the Vision subsystem.
     *
     * @param poseSupplier   Supplies the robot's current estimated pose.
     *                       In RobotContainer: () -> drivetrain.getState().Pose
     *
     * @param visionConsumer Accepts a vision pose to fuse into the estimator.
     *                       In RobotContainer: drivetrain::addVisionMeasurement
     *
     * @param omegaSupplier  Supplies the robot's angular velocity in RADIANS/SEC.
     *                       periodic() divides by (2π) to convert to rotations/sec.
     *                       In RobotContainer:
     *                         () -> drivetrain.getState().Speeds.omegaRadiansPerSecond
     */
    public Vision(
            Supplier<Pose2d> poseSupplier,
            VisionMeasurementConsumer visionConsumer,
            java.util.function.DoubleSupplier omegaSupplier) {

        setName("Vision");

        this.poseSupplier   = poseSupplier;
        this.visionConsumer = visionConsumer;
        this.omegaSupplier  = omegaSupplier;

        // Get NT handles so we can send commands to each camera (e.g. throttle_set).
        llATable = NetworkTableInstance.getDefault().getTable(LL_A);
        llBTable = NetworkTableInstance.getDefault().getTable(LL_B);

        // Start throttled immediately on boot so the cameras stay cool before
        // a match. Robot.java calls setThrottle(THROTTLE_ENABLED) when
        // autonomous or teleop starts.
        setThrottle(THROTTLE_DISABLED);
    }

    // ===== THROTTLE CONTROL =====
    /**
     * Sets the frame-processing rate on both cameras.
     *
     * Call from Robot.java:
     *   disabledInit()   → setThrottle(Vision.THROTTLE_DISABLED)
     *   autonomousInit() → setThrottle(Vision.THROTTLE_ENABLED)
     *   teleopInit()     → setThrottle(Vision.THROTTLE_ENABLED)
     *
     * @param throttle Frames to skip between each processed frame.
     *                 0 = full speed. 200 = process 1 of every 201 frames.
     */
    public void setThrottle(int throttle) {
        llATable.getEntry("throttle_set").setNumber(throttle);
        llBTable.getEntry("throttle_set").setNumber(throttle);
    }

    // ===== CORE LOGIC: PROCESS ONE LIMELIGHT =====
    /**
     * Reads one camera's MegaTag2 pose estimate and fuses it into odometry
     * if it passes all safety checks.
     *
     * Flow:
     *   1. Tell the camera our current heading (required for MegaTag2)
     *   2. If enabled: flush NetworkTables so the camera has the heading before
     *      we ask for its estimate in the same 20ms loop.
     *   3. Request the MegaTag2 pose estimate
     *   4. Safety checks: null, tag count, spin rate, field bounds
     *   5. Fuse into odometry with distance-scaled trust level
     *   6. Publish raw pose and return it for PoseError calculation in periodic()
     *
     * @param limelightName  Network name of this camera (e.g., "limelight-a")
     * @param currentHeading Robot heading in degrees from the gyro
     * @param omegaRps       Robot spin rate in rotations/sec
     * @param acceptedPub    NT publisher: did this camera contribute this loop?
     * @param tagCountPub    NT publisher: how many tags MegaTag2 used this loop
     * @param avgDistPub     NT publisher: average distance to visible tags (meters)
     * @param rawPoseXPub    NT publisher: this camera's raw X estimate (accepted only)
     * @param rawPoseYPub    NT publisher: this camera's raw Y estimate (accepted only)
     * @return The accepted Pose2d if all checks passed, or null if rejected
     */
    private Pose2d processLimelight(
            String limelightName,
            double currentHeading,
            double omegaRps,
            BooleanPublisher acceptedPub,
            DoublePublisher tagCountPub,
            DoublePublisher avgDistPub,
            DoublePublisher rawPoseXPub,
            DoublePublisher rawPoseYPub) {

        // Step 1: Tell the Limelight our current heading.
        // MegaTag2 NEEDS this to produce an accurate pose estimate.
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentHeading,  // yaw in degrees (CCW positive, 0 = facing red wall)
            0, 0, 0, 0, 0); // pitch, pitch rate, roll, roll rate, yaw rate (zeros fine)

        // Step 2: Flush NetworkTables (while enabled only).
        // Forces the heading we just wrote to reach the camera before we ask
        // for its pose estimate in the same 20ms loop.
        // Skipped while disabled to avoid unnecessary network traffic.
        if (!DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().flush();
        }

        // Step 3: Get the MegaTag2 pose estimate.
        // "_wpiBlue" = coordinate origin is always the blue alliance corner,
        // regardless of which alliance we're on. Correct for 2024 onward.
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // ======= Safety checks =======
        // Each check either lets the data through or rejects it.

        // Check A: Did the camera return anything at all?
        // mt2 is null if the camera isn't connected or hasn't published data yet.
        if (mt2 == null) {
            acceptedPub.set(false);
            tagCountPub.set(0);
            avgDistPub.set(-1);
            return null;
        }

        // Publish raw counts BEFORE filtering, so the dashboard shows real camera
        // data even when we ultimately reject the measurement below.
        tagCountPub.set(mt2.tagCount);
        avgDistPub.set(mt2.avgTagDist);

        // Check B: Did MegaTag2 use any tags in its pose solve?
        if (mt2.tagCount == 0) {
            acceptedPub.set(false);
            return null;
        }

        // Check C: Is the robot spinning too fast for reliable vision?
        // Fast rotation blurs the tag image, making pose estimates noisy.
        if (Math.abs(omegaRps) > MAX_ANGULAR_VELOCITY_RPS) {
            acceptedPub.set(false);
            return null;
        }

        // Check D: Is the estimated pose actually on the field?
        Pose2d visionPose = mt2.pose;
        if (visionPose.getX() < -FIELD_MARGIN_METERS
                || visionPose.getX() > FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
                || visionPose.getY() < -FIELD_MARGIN_METERS
                || visionPose.getY() > FIELD_WIDTH_METERS  + FIELD_MARGIN_METERS) {
            acceptedPub.set(false);
            return null;
        }

        // Step 4: Calculate how much to trust this measurement.
        // More tags = more trust. Greater distance = less trust.
        Matrix<N3, N1> stdDevs;
        if (mt2.tagCount >= 2) {
            stdDevs = MULTI_TAG_STD_DEVS;
        } else {
            stdDevs = SINGLE_TAG_STD_DEVS;
        }

        // Scale uncertainty by distance: farther away = less accurate.
        // At 5.5m: scale = 1 + (5.5² / 30) ≈ 2.0 → std devs double.
        double distanceScale = 1.0 + (mt2.avgTagDist * mt2.avgTagDist / 30.0);
        stdDevs = stdDevs.times(distanceScale);

        // Step 5: Send to the drivetrain's Kalman filter.
        visionConsumer.accept(visionPose, mt2.timestampSeconds, stdDevs);

        // Step 6: Publish this camera's raw X/Y for scalar telemetry.
        // Only written on accepted frames — no garbage on rejects.
        rawPoseXPub.set(visionPose.getX());
        rawPoseYPub.set(visionPose.getY());

        acceptedPub.set(true);

        // Return the accepted pose so periodic() can publish the 2D ghost robot
        // and compute PoseErrorMeters.
        return visionPose;
    }

    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // We intentionally process vision while disabled so the robot's pose
        // stays current before autonomous begins. Heat is managed by
        // THROTTLE_DISABLED = 200 on the camera hardware side.

        Pose2d currentPose = poseSupplier.get();
        double headingDeg  = currentPose.getRotation().getDegrees();
        sentHeadingPub.set(headingDeg); 

        // Convert from radians/sec → rotations/sec by dividing by 2π.
        double omegaRps = omegaSupplier.getAsDouble() / (2 * Math.PI);

        // ---- Process each camera independently ----
        // Each one can accept or reject its own measurement every loop.
        Pose2d aRawPose = processLimelight(
            LL_A, headingDeg, omegaRps,
            aAcceptedPub, aTagCountPub, aAvgDistPub,
            aRawPoseXPub, aRawPoseYPub);

        Pose2d bRawPose = processLimelight(
            LL_B, headingDeg, omegaRps,
            bAcceptedPub, bTagCountPub, bAvgDistPub,
            bRawPoseXPub, bRawPoseYPub);

        // ---- Publish 2D Field ghost robots ----
        // Each array is [x, y, rotationDegrees] — the format AdvantageScope
        // expects when you add the topic as a "Robot" on the 2D Field view.
        //
        // Camera poses only update on accepted frames so the ghost "freezes"
        // at the last good position instead of jumping to 0,0 on a reject.
        if (aRawPose != null) {
            aPosePub.set(new double[] {
                aRawPose.getX(),
                aRawPose.getY(),
                aRawPose.getRotation().getDegrees()
            });
        }

        if (bRawPose != null) {
            bPosePub.set(new double[] {
                bRawPose.getX(),
                bRawPose.getY(),
                bRawPose.getRotation().getDegrees()
            });
        }

        // Blended pose always updates — it's always available from the drivetrain.
        // NOTE: uses blendedPosePub, NOT bPosePub (that's camera B above).
        blendedPosePub.set(new double[] {
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getDegrees()
        });

        // ---- Publish scalar comparison telemetry ----
        odoPoseXPub.set(currentPose.getX());
        odoPoseYPub.set(currentPose.getY());

        // PoseLocked = true only when BOTH cameras accepted a reading this loop.
        boolean aAccepted = (aRawPose != null);
        boolean bAccepted = (bRawPose != null);
        poseLockedPub.set(aAccepted && bAccepted);

        // PoseErrorMeters = straight-line distance between camera A's raw reading
        // and the current blended pose. Published as -1.0 when camera A has no
        // accepted reading this loop.
        if (aAccepted) {
            double deltaX = aRawPose.getX() - currentPose.getX();
            double deltaY = aRawPose.getY() - currentPose.getY();
            poseErrorPub.set(Math.hypot(deltaX, deltaY));
        } else {
            poseErrorPub.set(-1.0);
        }
    }
}