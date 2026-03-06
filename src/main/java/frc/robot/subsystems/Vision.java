package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
 *   what each camera independently measured vs. what odometry has settled on:
 *
 *     Vision/A_RawPoseX, Y   → Where camera A thinks the robot is (meters)
 *     Vision/B_RawPoseX, Y   → Where camera B thinks the robot is (meters)
 *     Vision/OdoPoseX, Y     → Where blended odometry (wheels + vision) thinks we are
 *     Vision/PoseErrorMeters → Straight-line distance between camera A and odometry
 *     Vision/PoseLocked      → true when BOTH cameras accepted readings this loop
 *
 *   In AdvantageScope: add Vision/A_RawPoseX + A_RawPoseY to the Odometry tab
 *   to see a "ghost robot" showing where camera A alone thinks you are.
 *
 * PROCESSLIMELLIGHT RETURN TYPE:
 *   processLimelight() returns the accepted Pose2d, or null if rejected.
 *   This lets periodic() compute PoseErrorMeters cleanly without needing
 *   extra class fields or a second camera read.
 *   Think of it like: instead of a guard just saying "approved" (boolean),
 *   the guard hands you the actual visitor badge (Pose2d) to read the details.
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
    // This is separate from LimelightHelpers, which reads pose data FROM the cameras.
    private final NetworkTable llATable;
    private final NetworkTable llBTable;

    // ===== THROTTLE CONSTANTS =====
    // throttle_set tells the Limelight how many frames to SKIP between each
    // frame it actually processes for pose estimation.
    //
    // Analogy: THROTTLE_ENABLED = camera sprinting at full speed during a match.
    //          THROTTLE_DISABLED = camera barely jogging in the pits to stay cool.
    //
    //   THROTTLE_ENABLED  = 0   → process every frame (full speed during a match)
    //   THROTTLE_DISABLED = 200 → process 1 frame, skip 200, process 1, skip 200...
    //
    // We still run periodic() every 20ms while disabled — this is intentional,
    // so the robot's pose stays current before autonomous begins. The roboRIO
    // just reads a mostly-stale NT value each loop, which is nearly free.
    // The camera doing actual vision math is the expensive part, and throttle
    // controls that on the camera side.
    //
    // Think of it like checking your mailbox every day (cheap) vs. the mail
    // carrier driving the full route every day (expensive). Throttle tells the
    // mail carrier to only run the route once every few minutes.
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
    // poseSupplier:  asks the drivetrain "where do you think we are right now?"
    // visionConsumer: tells the drivetrain "vision thinks we're HERE at THIS time"
    // omegaSupplier: asks the drivetrain "how fast are we spinning?" (radians/sec)
    //                Divided by (2π) in periodic() to convert to rotations/sec.
    private final Supplier<Pose2d> poseSupplier;
    private final VisionMeasurementConsumer visionConsumer;
    private final java.util.function.DoubleSupplier omegaSupplier;

    // =========================================================================
    // ===== NETWORKTABLES PUBLISHERS ==========================================
    // =========================================================================
    // Think of publishers like the robot's "outbox" — written every 20ms,
    // read by Elastic and AdvantageScope. One-way: robot → dashboard.

    // Per-camera accepted/rejected flag
    // true  = reading passed all checks and was sent to the drivetrain this loop
    // false = rejected (camera offline, no tags, spinning too fast, or off-field)
    private final BooleanPublisher aAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/A_Accepted").publish();
    private final BooleanPublisher bAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/B_Accepted").publish();

    // How many AprilTags each camera's MegaTag2 solve used this loop
    private final DoublePublisher aTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_TagCount").publish();
    private final DoublePublisher bTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_TagCount").publish();

    // Average distance (meters) to the tags each camera currently sees
    private final DoublePublisher aAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_AvgDist").publish();
    private final DoublePublisher bAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_AvgDist").publish();

    // =========================================================================
    // ===== RAW POSE COMPARISON PUBLISHERS ====================================
    // =========================================================================
    // These show what each camera independently measured BEFORE it gets blended
    // into the drivetrain's estimate. Only written on ACCEPTED frames — no
    // garbage values on rejects. The last good value "sticks" on the dashboard
    // until a new accepted reading comes in.
    //
    // Analogy: two GPS units each give you their individual reading.
    // "Odometry" is the navigator's final answer after weighing both GPSes
    // plus the wheel sensors. These publishers show each GPS's raw opinion.

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
    // Compare against A_RawPoseX/Y and B_RawPoseX/Y to see whether the cameras agree.
    private final DoublePublisher odoPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/OdoPoseX").publish();
    private final DoublePublisher odoPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/OdoPoseY").publish();

    // Straight-line distance (meters) between camera A's raw reading and odometry.
    // "How much do the camera and wheel-tracking currently disagree?"
    //
    //   < 0.2 m   = great
    //   0.2–0.5 m = ok, normal during fast motion
    //   0.5–1.0 m = investigate (was starting pose reset correctly?)
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

        this.poseSupplier  = poseSupplier;
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
     *   2. Flush NetworkTables so the camera receives the heading before we
     *      ask for its estimate in the same 20ms loop
     *   3. Request the MegaTag2 pose estimate
     *   4. Safety checks: null, tag count, spin rate, field bounds
     *   5. Fuse into odometry with distance-scaled trust level
     *   6. Publish raw pose and return it for PoseError calculation in periodic()
     *
     * WHY rawPoseXPub / rawPoseYPub are parameters:
     *   This method is called once for camera A and once for camera B.
     *   Each call needs to write to its OWN publisher (A_RawPoseX vs B_RawPoseX).
     *   Passing the publisher as a parameter avoids duplicating the whole method.
     *   Think of it like a fill-in-the-blank form — same form, different name on top.
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
        // MegaTag2 NEEDS this — it fuses the gyro heading with what it sees
        // to produce a much more accurate pose estimate. Without it, MegaTag2
        // can't resolve the tag's 3D orientation and returns no pose at all.
        // Think of it like telling a navigator which direction your car is
        // facing before asking where you are — that context makes a big difference.
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentHeading,  // yaw in degrees (CCW positive, 0 = facing red wall)
            0, 0, 0, 0, 0); // pitch, pitch rate, roll, roll rate, yaw rate (zeros fine)

        // Step 2: Flush NetworkTables.
        // SetRobotOrientation writes to NT asynchronously — like sending a text.
        // If we call getBotPoseEstimate immediately in the same 20ms loop,
        // the camera might still use last loop's heading when building the estimate.
        // flush() forces all pending NT writes out over the wire right now, so the
        // camera has the current heading before we ask for the result.
        //
        // Without this, cameras can return tagCount = 0 even while their LEDs
        // are blinking fast (showing they physically see tags). The LEDs reflect
        // the raw detection pipeline; tagCount reflects the MegaTag2 pose solver —
        // they are separate systems inside the camera and can disagree.
        NetworkTableInstance.getDefault().flush();

        // Step 3: Get the MegaTag2 pose estimate.
        // "_wpiBlue" = coordinate origin is always the blue alliance corner,
        // regardless of which alliance we're on. Correct for 2024 onward.
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // ======= Safety checks =======
        // Each check below either lets the data through or rejects it.
        // Rejected data: publish false, return null.

        // Check A: Did the camera return anything at all?
        // mt2 is null if the camera isn't connected or hasn't published data yet.
        if (mt2 == null) {
            acceptedPub.set(false);
            tagCountPub.set(0);
            avgDistPub.set(-1);
            return null;
        }

        // Publish raw counts BEFORE filtering, so Elastic shows real camera data
        // even when we ultimately reject the measurement below.
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
        // If vision claims we're 30 meters away, something went wrong.
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
        // Think of std devs like error bars: 0.5m means "accurate to ~half a meter."
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

        // Step 5: Send to the drivetrain.
        // We pass mt2.timestampSeconds directly.
        // CommandSwerveDrivetrain.addVisionMeasurement() handles the one and only
        // FPGA → CTRE timestamp conversion internally. Do not wrap this in
        // Utils.fpgaToCurrentTime() — that would convert it twice and produce a
        // garbage timestamp that CTRE's Kalman filter silently discards.
        visionConsumer.accept(visionPose, mt2.timestampSeconds, stdDevs);

        // Step 6: Publish this camera's raw pose for dashboard comparison.
        // Only written on accepted frames — no garbage on rejects.
        // rawPoseXPub and rawPoseYPub are whichever publishers were passed in:
        //   Camera A call → aRawPoseXPub, aRawPoseYPub → writes to Vision/A_RawPoseX, Y
        //   Camera B call → bRawPoseXPub, bRawPoseYPub → writes to Vision/B_RawPoseX, Y
        rawPoseXPub.set(visionPose.getX());
        rawPoseYPub.set(visionPose.getY());

        acceptedPub.set(true);

        // Return the accepted pose so periodic() can compute PoseErrorMeters.
        return visionPose;
    }

    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // We intentionally process vision while disabled so the robot's pose
        // stays current before autonomous begins. The roboRIO reading a
        // mostly-stale NT value every 20ms is nearly free. Camera heat is
        // managed by THROTTLE_DISABLED = 200 on the camera hardware side.

        Pose2d currentPose = poseSupplier.get();
        double headingDeg  = currentPose.getRotation().getDegrees();

        // Convert from radians/sec → rotations/sec by dividing by 2π.
        // There are 2π radians in one full rotation — same idea as converting
        // miles/hour to km/hour, just a different unit scale.
        double omegaRps = omegaSupplier.getAsDouble() / (2 * Math.PI);

        // Process each camera independently.
        // Each one can accept or reject its own measurement every loop.
        Pose2d aRawPose = processLimelight(
            LL_A, headingDeg, omegaRps,
            aAcceptedPub, aTagCountPub, aAvgDistPub,
            aRawPoseXPub, aRawPoseYPub);

        Pose2d bRawPose = processLimelight(
            LL_B, headingDeg, omegaRps,
            bAcceptedPub, bTagCountPub, bAvgDistPub,
            bRawPoseXPub, bRawPoseYPub);

        // =====================================================================
        // Publish comparison telemetry
        // =====================================================================

        // Publish the blended odometry pose so you can compare it against the
        // raw camera poses in AdvantageScope or Elastic.
        odoPoseXPub.set(currentPose.getX());
        odoPoseYPub.set(currentPose.getY());

        // PoseLocked = true only when BOTH cameras accepted a reading this loop.
        // null = rejected, non-null = accepted.
        // Think of it like checking whether both referees held up a green flag.
        boolean aAccepted = (aRawPose != null);
        boolean bAccepted = (bRawPose != null);
        poseLockedPub.set(aAccepted && bAccepted);

        // PoseErrorMeters = straight-line distance between camera A's raw reading
        // and the current odometry pose, using the Pythagorean theorem.
        //
        //   Example: camera A says (3.2, 4.1), odometry says (3.0, 4.0).
        //     deltaX = 0.2m, deltaY = 0.1m → error = sqrt(0.04 + 0.01) ≈ 0.22m
        //
        //   Math.hypot(deltaX, deltaY) is a built-in shortcut for sqrt(deltaX² + deltaY²).
        //   It's the same formula as finding the diagonal length of a rectangle.
        //
        // Published as -1.0 when camera A has no accepted reading this loop.
        if (aAccepted) {
            double deltaX = aRawPose.getX() - currentPose.getX();
            double deltaY = aRawPose.getY() - currentPose.getY();
            poseErrorPub.set(Math.hypot(deltaX, deltaY));
        } else {
            poseErrorPub.set(-1.0);
        }
    }
}