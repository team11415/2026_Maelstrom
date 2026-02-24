package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;

// Math imports
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;

// CTRE timestamp utility
// CTRE uses a different clock than WPILib. Limelight gives us
// FPGA timestamps, but CTRE's addVisionMeasurement needs them
// converted. Think of it like converting time zones.
import com.ctre.phoenix6.Utils;

// Limelight library — this is the single file you downloaded
import frc.robot.LimelightHelpers;

import java.util.function.Supplier;


/**
 * Vision subsystem using two Limelight 4 cameras for pose estimation.
 * 
 * HOW THIS WORKS (the big picture):
 * 
 * Your old code (PhotonVision) worked like this:
 *   Camera sends raw images → Java code on roboRIO does the math → pose estimate
 * 
 * Limelight native software works like this:
 *   Camera sees tags → Camera's own processor does the math → publishes result to NetworkTables → we just read it
 * 
 * Think of it like the difference between doing your taxes yourself (PhotonVision)
 * vs hiring an accountant (Limelight) — you just get the final answer.
 * 
 * We use MegaTag2, which is Limelight's best mode. It combines:
 *   - What the camera sees (AprilTags)
 *   - What your gyro says (robot heading)
 * This eliminates the "tag flipping" problem entirely and works great
 * even with a single tag visible.
 */
public class Vision extends SubsystemBase {

    // ===== LIMELIGHT NAMES =====
    // These must match exactly what you set in the Limelight web UI.
    // You access the UI at http://limelight-a.local:5801
    private static final String LL_A = "limelight-a";
    private static final String LL_B = "limelight-b";

    // ===== FIELD DIMENSIONS =====
    // Used to reject poses that are "off the field" — clearly wrong.
    private static final double FIELD_LENGTH_METERS = 16.54;
    private static final double FIELD_WIDTH_METERS  = 8.21;
    private static final double FIELD_MARGIN_METERS = 0.5;

    // ===== SAFEGUARD CONSTANTS =====

    // If the robot is spinning faster than this (in rotations per second),
    // don't trust vision — the image is probably blurry.
    // 2.0 rotations/sec = 720 degrees/sec
    private static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;

    // ===== STANDARD DEVIATIONS =====
    // These control how much the drivetrain "trusts" vision vs. wheel odometry.
    //
    //   Small number = "I really trust this measurement"
    //   Big number   = "I'm skeptical of this measurement"
    //
    // Format: {x meters, y meters, rotation radians}
    //
    // With MegaTag2, rotation is handled by the gyro, so we set
    // rotation std dev to a huge number (9999999) meaning
    // "don't trust vision for rotation at all — trust the gyro."

    // When we see multiple tags — high confidence
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, 9999999);

    // When we see only one tag — lower confidence
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(1.0, 1.0, 9999999);

    // ===== CALLBACKS =====
    // poseSupplier: asks the drivetrain "where do you think we are?"
    // visionConsumer: tells the drivetrain "vision thinks we're HERE"
    private final Supplier<Pose2d> poseSupplier;
    private final VisionMeasurementConsumer visionConsumer;

    // ===== NETWORKTABLES PUBLISHERS =====
    // These let you see what's happening in AdvantageScope
    private final BooleanPublisher aAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/A_Accepted").publish();
    private final BooleanPublisher bAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/B_Accepted").publish();
    private final DoublePublisher aTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_TagCount").publish();
    private final DoublePublisher bTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_TagCount").publish();
    private final DoublePublisher aAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_AvgDist").publish();
    private final DoublePublisher bAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_AvgDist").publish();


    // ===== FUNCTIONAL INTERFACE =====
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
    }


    // ===== CORE LOGIC: PROCESS ONE LIMELIGHT =====
    /**
     * Processes one Limelight camera's pose estimate.
     * 
     * The flow:
     *   1. Tell the Limelight what direction the robot is facing (from the gyro)
     *   2. Ask it for a MegaTag2 pose estimate
     *   3. Run safeguard checks
     *   4. If it passes, send it to the drivetrain with appropriate trust level
     * 
     * @param limelightName  The network name of the Limelight (e.g., "limelight-a")
     * @param currentHeading The robot's current heading in degrees (from the gyro)
     * @param omegaRps       How fast the robot is spinning (rotations per second)
     * @param acceptedPub    NetworkTables publisher for debug telemetry
     * @param tagCountPub    NetworkTables publisher for debug telemetry
     * @param avgDistPub     NetworkTables publisher for debug telemetry
     * @return true if the measurement was accepted, false if rejected
     */
    private boolean processLimelight(
            String limelightName,
            double currentHeading,
            double omegaRps,
            BooleanPublisher acceptedPub,
            DoublePublisher tagCountPub,
            DoublePublisher avgDistPub) {

        // Step 1: Tell the Limelight our current heading.
        // MegaTag2 NEEDS this — it fuses the gyro heading with
        // what it sees to get a much better pose estimate.
        // The other parameters (pitch rate, yaw rate, etc.) are
        // set to 0 since we don't have those readily available.
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentHeading,  // yaw in degrees
            0, 0, 0, 0, 0); // pitch, yaw rate, etc. (zeros are fine)

        // Step 2: Get the MegaTag2 pose estimate.
        // "_wpiBlue" means the pose is in the WPILib Blue Alliance
        // coordinate system (which is what CTRE and PathPlanner use).
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Step 3: Safety checks

        // Check 3a: Did we get a result at all?
        // (mt2 can be null if the camera isn't connected or hasn't
        //  published data yet)
        if (mt2 == null) {
            acceptedPub.set(false);
            tagCountPub.set(0);
            avgDistPub.set(-1);
            return false;
        }

        // Publish how many tags we see (useful for debugging)
        tagCountPub.set(mt2.tagCount);
        avgDistPub.set(mt2.avgTagDist);

        // Check 3b: Did we see any tags?
        if (mt2.tagCount == 0) {
            acceptedPub.set(false);
            return false;
        }

        // Check 3c: Is the robot spinning too fast?
        // Fast rotation = blurry image = unreliable estimate
        if (Math.abs(omegaRps) > MAX_ANGULAR_VELOCITY_RPS) {
            acceptedPub.set(false);
            return false;
        }

        // Check 3d: Is the pose on the field?
        // If vision says we're 30 meters away, something is clearly wrong.
        Pose2d visionPose = mt2.pose;
        if (visionPose.getX() < -FIELD_MARGIN_METERS
                || visionPose.getX() > FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
                || visionPose.getY() < -FIELD_MARGIN_METERS
                || visionPose.getY() > FIELD_WIDTH_METERS + FIELD_MARGIN_METERS) {
            acceptedPub.set(false);
            return false;
        }

        // Step 4: Calculate how much to trust this measurement.
        // More tags = more trust, and we scale by distance.
        Matrix<N3, N1> stdDevs;
        if (mt2.tagCount >= 2) {
            stdDevs = MULTI_TAG_STD_DEVS;
        } else {
            stdDevs = SINGLE_TAG_STD_DEVS;
        }

        // Scale trust by distance: farther away = less trust
        double distanceScale = 1.0 + (mt2.avgTagDist * mt2.avgTagDist / 30.0);
        stdDevs = stdDevs.times(distanceScale);

        // Step 5: Send it to the drivetrain!
        // CRITICAL: Convert the timestamp from FPGA time to CTRE time.
        // Without this, CTRE will either ignore the measurement or
        // apply it at the wrong time. This is the #1 gotcha with
        // CTRE swerve + vision.
        double timestamp = Utils.fpgaToCurrentTime(mt2.timestampSeconds);
        visionConsumer.accept(visionPose, timestamp, stdDevs);

        acceptedPub.set(true);
        return true;
    }


    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // Get the current robot state from the drivetrain.
        // We need the heading for MegaTag2 and the angular velocity
        // for our "spinning too fast" safeguard.
        Pose2d currentPose = poseSupplier.get();
        double headingDeg = currentPose.getRotation().getDegrees();

        // We can't easily get angular velocity from just the pose supplier,
        // so we'll skip that check for now by passing 0.
        // TODO: If you want the angular velocity check, pass it from
        //       drivetrain.getState().Speeds.omegaRadiansPerSecond
        //       converted to rotations per second.
        double omegaRps = 0; // Placeholder — see TODO above

        // Process each camera independently.
        // Each one can accept or reject its own measurement.
        processLimelight(LL_A, headingDeg, omegaRps,
            aAcceptedPub, aTagCountPub, aAvgDistPub);

        processLimelight(LL_B, headingDeg, omegaRps,
            bAcceptedPub, bTagCountPub, bAvgDistPub);
    }
}