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
 *   Camera sees tags → Camera's own processor does the math →
 *   publishes result to NetworkTables → we just read it
 *
 * Think of it like the difference between doing your taxes yourself (PhotonVision)
 * vs hiring an accountant (Limelight) — you just get the final answer.
 *
 * We use MegaTag2, which is Limelight's best mode. It combines:
 *   - What the camera sees (AprilTags)
 *   - What your gyro says (robot heading)
 * This eliminates the "tag flipping" problem entirely and works great
 * even with a single tag visible.
 *
 * NEW IN THIS VERSION: Raw pose comparison telemetry.
 * You can now see in AdvantageScope or Elastic:
 *
 *   Vision/A_RawPoseX, Y   → Where camera A thinks the robot is (meters)
 *   Vision/B_RawPoseX, Y   → Where camera B thinks the robot is (meters)
 *   Vision/OdoPoseX, Y     → Where odometry (wheels + vision blended) thinks we are
 *   Vision/PoseErrorMeters → Straight-line distance between camera A and odometry
 *   Vision/PoseLocked      → true when BOTH cameras are accepting readings
 *
 * KEY DESIGN CHANGE FROM OLD VERSION:
 *   processLimelight() now returns Pose2d instead of boolean.
 *   - If accepted: returns the raw camera pose (the X/Y the camera measured)
 *   - If rejected: returns null (Java's way of saying "no valid result this frame")
 *   This lets periodic() compute the error distance cleanly without needing
 *   extra fields or a second call to the camera.
 */
public class Vision extends SubsystemBase {

    // ===== LIMELIGHT NAMES =====
    // These must match exactly what you set in the Limelight web UI.
    // You access the UI at http://limelight-a.local:5801
    private static final String LL_A = "limelight-a";
    private static final String LL_B = "limelight-b";

    // ===== NETWORKTABLES HANDLES =====
    // These are our "phone lines" to each Limelight camera.
    // We use them to send commands TO the cameras (like throttle_set).
    // This is separate from LimelightHelpers, which reads data FROM the cameras.
    private final NetworkTable llATable;
    private final NetworkTable llBTable;

    // ===== THROTTLE CONSTANTS =====
    // throttle_set tells the Limelight to skip N frames between each processed frame.
    //
    // Think of it like this:
    //   THROTTLE_ENABLED  = 0   → process every frame (full speed during a match)
    //   THROTTLE_DISABLED = 100 → skip 100 frames, process 1, skip 100, process 1...
    //
    // The Limelight docs recommend 50-200 while disabled to reduce heat.
    public static final int THROTTLE_ENABLED  = 0;
    public static final int THROTTLE_DISABLED = 100;

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
    // Small number = "I really trust this measurement"
    // Big number   = "I'm skeptical of this measurement"
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
    // poseSupplier:   asks the drivetrain "where do you think we are?"
    // visionConsumer: tells the drivetrain "vision thinks we're HERE"
    private final Supplier<Pose2d> poseSupplier;
    private final VisionMeasurementConsumer visionConsumer;

    // This asks the drivetrain "how fast are you spinning?" (in radians/sec)
    private final java.util.function.DoubleSupplier omegaSupplier;

    // =========================================================================
    // ===== NETWORKTABLES PUBLISHERS ==========================================
    // =========================================================================
    // Think of publishers like the robot's "outbox" — the robot writes to them
    // every 20ms and your dashboard reads them. Publishers are one-way: robot → dashboard.

    // ----- EXISTING: per-camera accepted/rejected status -----
    // true  = this camera's reading passed all checks and was sent to the drivetrain
    // false = rejected (null result, spinning too fast, no tags, or off-field)
    private final BooleanPublisher aAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/A_Accepted").publish();
    private final BooleanPublisher bAcceptedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/B_Accepted").publish();

    // ----- EXISTING: how many AprilTags each camera currently sees -----
    private final DoublePublisher aTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_TagCount").publish();
    private final DoublePublisher bTagCountPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_TagCount").publish();

    // ----- EXISTING: average distance (meters) to the visible tags -----
    private final DoublePublisher aAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_AvgDist").publish();
    private final DoublePublisher bAvgDistPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_AvgDist").publish();

    // =========================================================================
    // ===== NEW: RAW POSE COMPARISON PUBLISHERS ================================
    // =========================================================================
    // These show you what each camera independently measured, BEFORE it gets
    // blended into the drivetrain's estimate.
    //
    // Analogy: Imagine two GPS units each give you a location.
    // The "odometry" is the navigator's final conclusion after weighing both GPSes
    // plus the wheel sensors. These raw publishers show each GPS's individual reading.
    //
    // HOW TO USE IN ADVANTAGESCOPE:
    //   Go to the Odometry tab → click "+" → add Vision/A_RawPoseX and A_RawPoseY
    //   This shows a second "ghost robot" at where camera A thinks you are.
    //   If the ghost is close to the real robot, camera A is trustworthy.
    //
    // HOW TO USE IN ELASTIC:
    //   Add Number Bar or Line Graph widgets for Vision/A_RawPoseX, Vision/OdoPoseX, etc.

    // Camera A's raw pose estimate (X and Y in meters, field coordinate system).
    // Only updated when camera A ACCEPTS its reading — no garbage published on rejects.
    private final DoublePublisher aRawPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_RawPoseX").publish();
    private final DoublePublisher aRawPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/A_RawPoseY").publish();

    // Camera B's raw pose estimate (X and Y in meters, field coordinate system).
    private final DoublePublisher bRawPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_RawPoseX").publish();
    private final DoublePublisher bRawPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/B_RawPoseY").publish();

    // The drivetrain's current blended odometry pose (wheels + all accepted vision updates).
    // This is the position the robot actually uses for aiming, auto paths, etc.
    // Compare against A_RawPoseX/Y and B_RawPoseX/Y to see if the cameras agree.
    private final DoublePublisher odoPoseXPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/OdoPoseX").publish();
    private final DoublePublisher odoPoseYPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/OdoPoseY").publish();

    // Straight-line distance (meters) between camera A's raw reading and odometry.
    // This tells you: "how much do the camera and wheel-tracking disagree?"
    //
    // Good thresholds:
    //   < 0.2m  = great, cameras and odometry agree closely
    //   0.2–0.5m = ok, normal when driving quickly or robot just moved
    //   0.5–1.0m = investigate — maybe initial pose wasn't reset correctly
    //   > 1.0m  = problem — check camera pipeline or verify tag layout is correct
    //
    // Published as -1.0 when camera A has no accepted reading this frame.
    private final DoublePublisher poseErrorPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Vision/PoseErrorMeters").publish();

    // True when BOTH cameras accepted their most recent reading this frame.
    // This is your "green light" — both cameras locked on means high-confidence pose.
    // One camera = ok. Both cameras = great.
    private final BooleanPublisher poseLockedPub = NetworkTableInstance.getDefault()
        .getBooleanTopic("Vision/PoseLocked").publish();

    // =========================================================================

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

        this.poseSupplier   = poseSupplier;
        this.visionConsumer = visionConsumer;
        this.omegaSupplier  = omegaSupplier;

        // Get a NetworkTable handle for each camera so we can send
        // commands to them (like throttle_set).
        llATable = NetworkTableInstance.getDefault().getTable(LL_A);
        llBTable = NetworkTableInstance.getDefault().getTable(LL_B);

        // Start throttled immediately on boot so cameras stay cool
        // before a match begins. Robot.java will call setThrottle(THROTTLE_ENABLED)
        // when autonomous or teleop starts.
        setThrottle(THROTTLE_DISABLED);
    }

    // ===== THROTTLE CONTROL =====
    /**
     * Sets the frame processing throttle on both Limelight cameras.
     *
     * Think of this like adjusting how hard the cameras are working:
     *   - THROTTLE_ENABLED  (0)   = full speed, process every frame (use during a match)
     *   - THROTTLE_DISABLED (100) = process 1 out of every 101 frames (use while disabled)
     *
     * Call this from Robot.java in disabledInit(), autonomousInit(), and teleopInit().
     * The constants THROTTLE_ENABLED and THROTTLE_DISABLED are public so Robot.java can use them.
     *
     * @param throttle Number of frames to skip between each processed frame.
     */
    public void setThrottle(int throttle) {
        llATable.getEntry("throttle_set").setNumber(throttle);
        llBTable.getEntry("throttle_set").setNumber(throttle);
    }

    // ===== CORE LOGIC: PROCESS ONE LIMELIGHT =====
    /**
     * Processes one Limelight camera's pose estimate.
     *
     * The flow:
     *   1. Tell the Limelight what direction the robot is facing (from the gyro)
     *   2. Ask it for a MegaTag2 pose estimate
     *   3. Run safeguard checks — reject if anything looks wrong
     *   4. If accepted: publish raw pose, send to drivetrain, return the Pose2d
     *   5. If rejected: return null
     *
     * KEY CHANGE FROM OLD VERSION:
     *   Old return type was "boolean" — just true (accepted) or false (rejected).
     *   New return type is "Pose2d" — the actual pose if accepted, or null if rejected.
     *
     *   Why change this? Because periodic() needs the raw pose NUMBER (not just yes/no)
     *   in order to compute PoseErrorMeters. Returning the pose directly avoids having
     *   to store it in a separate class field and keeps the code cleaner.
     *
     *   Think of it like: instead of a security guard just saying "approved" (boolean),
     *   the guard now hands you the visitor's badge (Pose2d) so you can read the details.
     *
     * WHY rawPoseXPub / rawPoseYPub are parameters instead of being hardcoded:
     *   This single method is called twice — once for camera A and once for camera B.
     *   Each camera needs to write to ITS OWN publisher (A_RawPoseX vs B_RawPoseX).
     *   By passing the publisher as a parameter, we avoid writing two nearly-identical methods.
     *   Think of it like a fill-in-the-blank form — same form, different name on top.
     *
     * @param limelightName  The network name of the Limelight (e.g., "limelight-a")
     * @param currentHeading The robot's current heading in degrees (from the gyro)
     * @param omegaRps       How fast the robot is spinning (rotations per second)
     * @param acceptedPub    Publisher: writes true/false for accepted/rejected
     * @param tagCountPub    Publisher: writes number of AprilTags visible
     * @param avgDistPub     Publisher: writes average distance to tags
     * @param rawPoseXPub    Publisher: writes this camera's raw X estimate (NEW)
     * @param rawPoseYPub    Publisher: writes this camera's raw Y estimate (NEW)
     * @return The accepted Pose2d if all checks passed, or null if rejected
     */
    private Pose2d processLimelight(
        String limelightName,
        double currentHeading,
        double omegaRps,
        BooleanPublisher acceptedPub,
        DoublePublisher tagCountPub,
        DoublePublisher avgDistPub,
        DoublePublisher rawPoseXPub,    // NEW parameter
        DoublePublisher rawPoseYPub) {  // NEW parameter

        // Step 1: Tell the Limelight our current heading.
        // MegaTag2 NEEDS this — it fuses the gyro heading with
        // what it sees to get a much better pose estimate.
        // The other parameters (pitch rate, yaw rate, etc.) are
        // set to 0 since we don't have those readily available.
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentHeading,   // yaw in degrees
            0, 0, 0, 0, 0);  // pitch, yaw rate, etc. (zeros are fine)

        // Step 2: Get the MegaTag2 pose estimate.
        // "_wpiBlue" means the pose is in the WPILib Blue Alliance
        // coordinate system (which is what CTRE and PathPlanner use).
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // ======= Step 3: Safety checks =======
        // Each check below either lets the data through or rejects it.
        // Rejected data means: publish false to acceptedPub, return null.

        // Check 3a: Did we get a result at all?
        // mt2 is null if the camera isn't connected or hasn't published data yet.
        if (mt2 == null) {
            acceptedPub.set(false);
            tagCountPub.set(0);
            avgDistPub.set(-1);
            return null; // ← null means "no valid result"
        }

        // Publish how many tags we see (useful for debugging)
        tagCountPub.set(mt2.tagCount);
        avgDistPub.set(mt2.avgTagDist);

        // Check 3b: Did we see any tags?
        // No tags = no position information = useless reading
        if (mt2.tagCount == 0) {
            acceptedPub.set(false);
            return null;
        }

        // Check 3c: Is the robot spinning too fast?
        // Fast rotation = motion blur = unreliable estimate.
        // 2.0 RPS = 720 degrees per second, which is very fast.
        if (Math.abs(omegaRps) > MAX_ANGULAR_VELOCITY_RPS) {
            acceptedPub.set(false);
            return null;
        }

        // Check 3d: Is the pose on the field?
        // If vision says we're 30 meters away, something is clearly wrong.
        // The 0.5m margin allows for being right at the boundary.
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

        // Scale trust by distance: farther away = less trust.
        // At 5.5m: distanceScale = 1 + (5.5² / 30) = 1 + 1.0 = 2.0
        //   meaning the std devs double, so the drivetrain trusts it half as much.
        double distanceScale = 1.0 + (mt2.avgTagDist * mt2.avgTagDist / 30.0);
        stdDevs = stdDevs.times(distanceScale);

        // Step 5: Send it to the drivetrain!
        // CRITICAL: Convert the timestamp from FPGA time to CTRE time.
        // Without this, CTRE will either ignore the measurement or apply it at
        // the wrong time. This is the #1 gotcha with CTRE swerve + vision.
        double timestamp = Utils.fpgaToCurrentTime(mt2.timestampSeconds);
        visionConsumer.accept(visionPose, timestamp, stdDevs);

        // Step 6: Publish this camera's raw pose for dashboard comparison.
        // This only runs when ACCEPTED — so you only see valid data on the graph.
        // The last accepted value "sticks" on the dashboard until a new one comes in.
        //
        // rawPoseXPub and rawPoseYPub are whichever publishers were passed in:
        //   - Camera A call → aRawPoseXPub, aRawPoseYPub → writes to Vision/A_RawPoseX, Y
        //   - Camera B call → bRawPoseXPub, bRawPoseYPub → writes to Vision/B_RawPoseX, Y
        rawPoseXPub.set(visionPose.getX());
        rawPoseYPub.set(visionPose.getY());

        acceptedPub.set(true);

        // Return the accepted pose so periodic() can use it for error calculation.
        // Old code returned 'true' here. Now we return the actual Pose2d object.
        return visionPose;
    }

    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // Get the current robot state from the drivetrain.
        // We need the heading for MegaTag2 and the angular velocity
        // for our "spinning too fast" safeguard.
        Pose2d currentPose = poseSupplier.get();
        double headingDeg  = currentPose.getRotation().getDegrees();

        // Convert from radians/second → rotations/second by dividing by 2π.
        // There are 2π radians in one full rotation — this is just a unit conversion,
        // like converting miles/hour to km/hour.
        double omegaRps = omegaSupplier.getAsDouble() / (2 * Math.PI);

        // Process each camera independently.
        // Each one can accept or reject its own measurement.
        //
        // WHAT CHANGED:
        //   Old: processLimelight(...) returned boolean, we ignored the return value
        //   New: processLimelight(...) returns Pose2d or null, we store the result
        //        This lets us compute PoseErrorMeters below.
        //
        // Camera A: pass aRawPoseXPub and aRawPoseYPub as the last two arguments
        Pose2d aRawPose = processLimelight(
            LL_A, headingDeg, omegaRps,
            aAcceptedPub, aTagCountPub, aAvgDistPub,
            aRawPoseXPub, aRawPoseYPub);  // ← NEW: camera A gets its own publishers

        // Camera B: pass bRawPoseXPub and bRawPoseYPub as the last two arguments
        Pose2d bRawPose = processLimelight(
            LL_B, headingDeg, omegaRps,
            bAcceptedPub, bTagCountPub, bAvgDistPub,
            bRawPoseXPub, bRawPoseYPub);  // ← NEW: camera B gets its own publishers

        // =====================================================================
        // ===== NEW: Publish comparison telemetry =============================
        // =====================================================================

        // Publish the odometry (blended) pose — what the robot is actually using.
        // In AdvantageScope, compare Vision/OdoPoseX against Vision/A_RawPoseX
        // to see whether camera A's reading matches where the robot thinks it is.
        odoPoseXPub.set(currentPose.getX());
        odoPoseYPub.set(currentPose.getY());

        // PoseLocked = true only when BOTH cameras accepted a reading this frame.
        // In Java, null means "no result" — so we check if each returned non-null.
        // This is like checking whether both referees held up a green flag.
        boolean aAccepted = (aRawPose != null);
        boolean bAccepted = (bRawPose != null);
        poseLockedPub.set(aAccepted && bAccepted);

        // PoseErrorMeters = straight-line distance between camera A's raw reading
        // and the current odometry pose.
        //
        // HOW THE MATH WORKS — the Pythagorean theorem:
        //   If camera A says robot is at (3.2, 4.1)
        //   and odometry says robot is at (3.0, 4.0), then:
        //     deltaX = 3.2 - 3.0 = 0.2m
        //     deltaY = 4.1 - 4.0 = 0.1m
        //     error  = sqrt(0.2² + 0.1²) = sqrt(0.05) ≈ 0.22m
        //
        //   Math.hypot(deltaX, deltaY) is a built-in shortcut for sqrt(deltaX² + deltaY²).
        //   It's the same formula you'd use to find the diagonal of a rectangle.
        //
        // We publish -1.0 when camera A was rejected this frame, as a "no data" signal.
        if (aAccepted) {
            // aRawPose is guaranteed non-null here because aAccepted is true
            double deltaX = aRawPose.getX() - currentPose.getX();
            double deltaY = aRawPose.getY() - currentPose.getY();
            poseErrorPub.set(Math.hypot(deltaX, deltaY));
        } else {
            // Camera A didn't accept this frame — no valid raw pose to compare against
            poseErrorPub.set(-1.0);
        }
    }
}