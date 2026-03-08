package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;

// Math imports
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// CTRE timestamp utility
// CTRE uses a different clock than WPILib. Limelight gives us
// FPGA timestamps, but CTRE's addVisionMeasurement needs them
// converted. Think of it like converting time zones.
import com.ctre.phoenix6.Utils;

// Limelight library — the single file you downloaded
import frc.robot.LimelightHelpers;

import java.util.function.Supplier;

/**
 * Vision subsystem using two Limelight 4 cameras for pose estimation.
 *
 * =====================================================================
 * NEW: ELASTIC CONTROL FLAGS (Requests #1 and #2)
 * =====================================================================
 * Three on/off switches are published to NetworkTables under the table
 * "ControlFlags". In Elastic, browse to that table and add each entry
 * as a "Toggle Switch" widget.
 *
 *   /ControlFlags/VisionEnabled   → master switch for ALL cameras
 *   /ControlFlags/CameraAEnabled  → just limelight-a
 *   /ControlFlags/CameraBEnabled  → just limelight-b
 *
 * All three default to TRUE (on) when robot code starts.
 *
 * Think of VisionEnabled like the main circuit breaker:
 *   - Flip it OFF → robot uses only the Pigeon + wheel odometry for position.
 *     The cameras are still running but the robot ignores their pose data.
 *   - CameraAEnabled / CameraBEnabled are individual breakers for each camera.
 *     Useful if one camera's view is blocked or giving bad data.
 * =====================================================================
 */
public class Vision extends SubsystemBase {

    // ===== LIMELIGHT NAMES =====
    // These must match exactly what you set in the Limelight web UI.
    // Access the UI at http://limelight-a.local:5801
    private static final String LL_A = "limelight-a";
    private static final String LL_B = "limelight-b";

    // ===== NETWORKTABLES HANDLES FOR CAMERA COMMANDS =====
    // Used to send throttle commands TO the cameras
    private final NetworkTable llATable;
    private final NetworkTable llBTable;

    // ===== ELASTIC CONTROL FLAG ENTRIES =====
    // NetworkTableEntry supports BOTH reading and writing, unlike
    // BooleanSubscriber (which our code can only read from).
    // Using NetworkTableEntry lets Elastic's toggle switch write the value,
    // and our code reads it each loop.
    //
    // Analogy: these are like light switches. Elastic is the hand flipping
    // the switch. Our code peeks at the switch state every 20ms.
    private final NetworkTableEntry visionEnabledEntry;   // master on/off
    private final NetworkTableEntry cameraAEnabledEntry;  // limelight-a only
    private final NetworkTableEntry cameraBEnabledEntry;  // limelight-b only

    // ===== THROTTLE CONSTANTS =====
    public static final int THROTTLE_ENABLED  = 0;   // process every frame (use during match)
    public static final int THROTTLE_DISABLED = 100; // skip most frames (use while disabled)

    // ===== FIELD DIMENSIONS =====
    private static final double FIELD_LENGTH_METERS = 16.54;
    private static final double FIELD_WIDTH_METERS  = 8.21;
    private static final double FIELD_MARGIN_METERS = 0.5;

    // ===== SAFEGUARD: MAX SPIN RATE =====
    // If the robot is spinning faster than this (rotations per second),
    // don't trust vision — the image is probably blurry.
    // 2.0 RPS = 720 degrees/sec
    private static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;

    // ===== STANDARD DEVIATIONS =====
    // Controls how much the drivetrain "trusts" vision vs. wheel odometry.
    // Small number = "I trust this a lot"  /  Big number = "I'm skeptical"
    // Format: {x meters, y meters, rotation radians}
    //
    // With MegaTag2, the gyro handles rotation, so we set rotation std dev
    // to a huge number (9999999) meaning "ignore vision rotation entirely."
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, 9999999);
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(1.0, 1.0, 9999999);

    // ===== CALLBACKS =====
    // poseSupplier:   asks the drivetrain "where do you think we are?"
    // visionConsumer: tells the drivetrain "vision thinks we're HERE"
    private final Supplier<Pose2d> poseSupplier;
    private final VisionMeasurementConsumer visionConsumer;
    private final java.util.function.DoubleSupplier omegaSupplier;

    // ===== NETWORKTABLES PUBLISHERS (AdvantageScope / Elastic telemetry) =====
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
        VisionMeasurementConsumer visionConsumer,
        java.util.function.DoubleSupplier omegaSupplier) {

        setName("Vision");
        this.poseSupplier  = poseSupplier;
        this.visionConsumer = visionConsumer;
        this.omegaSupplier  = omegaSupplier;

        // Get NetworkTable handles for each camera (for sending throttle commands)
        llATable = NetworkTableInstance.getDefault().getTable(LL_A);
        llBTable = NetworkTableInstance.getDefault().getTable(LL_B);

        // =========================================================
        // INITIALIZE ELASTIC CONTROL FLAGS
        // =========================================================
        // All three toggles live in the "ControlFlags" NetworkTable.
        //
        // setDefaultBoolean(true) means:
        //   "If nobody has set this entry yet, start it as TRUE."
        //   On every fresh code start, all cameras are enabled.
        //   A driver can flip the toggle in Elastic and it stays flipped
        //   until the code restarts — it won't be reset each loop.
        // =========================================================
        NetworkTable controlsTable = NetworkTableInstance.getDefault()
            .getTable("ControlFlags");

        visionEnabledEntry = controlsTable.getEntry("VisionEnabled");
        visionEnabledEntry.setDefaultBoolean(true);   // ON by default

        cameraAEnabledEntry = controlsTable.getEntry("CameraAEnabled");
        cameraAEnabledEntry.setDefaultBoolean(true);  // ON by default

        cameraBEnabledEntry = controlsTable.getEntry("CameraBEnabled");
        cameraBEnabledEntry.setDefaultBoolean(true);  // ON by default

        // Start throttled on boot so cameras don't overheat in the pits.
        // Robot.java calls setThrottle(THROTTLE_ENABLED) when auto/teleop starts.
        setThrottle(THROTTLE_DISABLED);
    }

    // ===== THROTTLE CONTROL =====
    /**
     * Sets the frame processing rate on both Limelight cameras.
     *   THROTTLE_ENABLED  (0)   = full speed — use this during a match
     *   THROTTLE_DISABLED (100) = very slow — use this when robot is disabled
     */
    public void setThrottle(int throttle) {
        llATable.getEntry("throttle_set").setNumber(throttle);
        llBTable.getEntry("throttle_set").setNumber(throttle);
    }

    // ===== CORE LOGIC: PROCESS ONE LIMELIGHT =====
    /**
     * Processes one Limelight camera's pose estimate and feeds it to the drivetrain.
     *
     *  1. Tell the Limelight our gyro heading (required for MegaTag2)
     *  2. Read the pose estimate
     *  3. Run safety checks (valid? on field? not spinning?)
     *  4. If passes, send to drivetrain with a calculated trust level
     *
     * @return true if the measurement was accepted and used, false if rejected
     */
    private boolean processLimelight(
        String limelightName,
        double currentHeading,
        double omegaRps,
        BooleanPublisher acceptedPub,
        DoublePublisher tagCountPub,
        DoublePublisher avgDistPub) {

        // Step 1: Tell the Limelight our current heading.
        // MegaTag2 REQUIRES this to fuse gyro + camera data together.
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentHeading,   // yaw in degrees
            0, 0, 0, 0, 0);   // pitch/roll rates (zeros are fine here)

        // Step 2: Get the MegaTag2 pose estimate.
        // "_wpiBlue" = WPILib Blue Alliance coordinates (what CTRE/PathPlanner use)
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Step 3: Safety checks — reject bad data before it can corrupt odometry

        // 3a: Did we get any data at all? (null = camera not connected or no data)
        if (mt2 == null) {
            acceptedPub.set(false);
            tagCountPub.set(0);
            avgDistPub.set(-1);
            return false;
        }

        tagCountPub.set(mt2.tagCount);
        avgDistPub.set(mt2.avgTagDist);

        // 3b: Did we actually see any AprilTags?
        if (mt2.tagCount == 0) {
            acceptedPub.set(false);
            return false;
        }

        // 3c: Is the robot spinning too fast? (blurry image = bad data)
        if (Math.abs(omegaRps) > MAX_ANGULAR_VELOCITY_RPS) {
            acceptedPub.set(false);
            return false;
        }

        // 3d: Is the pose physically on the field?
        // If vision says we're 30 meters away, something went very wrong.
        Pose2d visionPose = mt2.pose;
        if (visionPose.getX() < -FIELD_MARGIN_METERS
         || visionPose.getX() > FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
         || visionPose.getY() < -FIELD_MARGIN_METERS
         || visionPose.getY() > FIELD_WIDTH_METERS  + FIELD_MARGIN_METERS) {
            acceptedPub.set(false);
            return false;
        }

        // Step 4: Choose trust level — more tags = more confidence
        Matrix<N3, N1> stdDevs = (mt2.tagCount >= 2)
            ? MULTI_TAG_STD_DEVS
            : SINGLE_TAG_STD_DEVS;

        // Scale trust down for tags far away (farther = less accurate)
        double distanceScale = 1.0 + (mt2.avgTagDist * mt2.avgTagDist / 30.0);
        stdDevs = stdDevs.times(distanceScale);

        // Step 5: Send it to the drivetrain!
        // Convert FPGA timestamp to CTRE time — THIS IS CRITICAL.
        // Without this, CTRE ignores or misapplies the measurement.
        double timestamp = Utils.fpgaToCurrentTime(mt2.timestampSeconds);
        visionConsumer.accept(visionPose, timestamp, stdDevs);

        acceptedPub.set(true);
        return true;
    }

    // ===== PERIODIC =====
    // This runs every 20 milliseconds (50 times per second).
    @Override
    public void periodic() {

        // =====================================================
        // READ ELASTIC CONTROL FLAGS
        // =====================================================
        // The second argument to getBoolean() is the "fallback"
        // value if the entry is somehow missing. We default to
        // TRUE so that vision doesn't silently stop working if
        // something goes wrong with NetworkTables.

        boolean visionEnabled = visionEnabledEntry.getBoolean(true);

        // MASTER SWITCH CHECK:
        // If the whole vision system is off, publish "not accepted" to
        // telemetry so you can see it clearly in Elastic/AdvantageScope,
        // then bail out early — don't process either camera.
        if (!visionEnabled) {
            aAcceptedPub.set(false);
            bAcceptedPub.set(false);
            aTagCountPub.set(0);
            bTagCountPub.set(0);
            return; // ← EXIT EARLY: skip all vision this loop
        }

        // Per-camera toggle checks
        boolean cameraAEnabled = cameraAEnabledEntry.getBoolean(true);
        boolean cameraBEnabled = cameraBEnabledEntry.getBoolean(true);

        // Get robot state — needed for MegaTag2 heading and spin check
        Pose2d currentPose = poseSupplier.get();
        double headingDeg  = currentPose.getRotation().getDegrees();
        double omegaRps    = omegaSupplier.getAsDouble() / (2 * Math.PI);

        // --- Process Camera A ---
        if (cameraAEnabled) {
            processLimelight(LL_A, headingDeg, omegaRps,
                aAcceptedPub, aTagCountPub, aAvgDistPub);
        } else {
            // Camera A is off via Elastic toggle — update telemetry accordingly
            aAcceptedPub.set(false);
            aTagCountPub.set(0);
        }

        // --- Process Camera B ---
        if (cameraBEnabled) {
            processLimelight(LL_B, headingDeg, omegaRps,
                bAcceptedPub, bTagCountPub, bAvgDistPub);
        } else {
            // Camera B is off via Elastic toggle — update telemetry accordingly
            bAcceptedPub.set(false);
            bTagCountPub.set(0);
        }
    }
}