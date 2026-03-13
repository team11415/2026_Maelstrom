package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Publishes extra telemetry for testing and match monitoring.
 * This subsystem does NOT control anything — it only reads and reports.
 *
 * =====================================================================
 * BUG FIX: Duplicate Motor Handles
 * =====================================================================
 * The old version of this class created new TalonFX / TalonFXS objects
 * for CAN IDs that were already owned by Shooter.java and Spindexer.java:
 *
 *   OLD (broken):
 *     private final TalonFX shooterRight = new TalonFX(61, ...);  // also in Shooter!
 *     private final TalonFXS spindexerMotor = new TalonFXS(54, ...); // also in Spindexer!
 *     private final TalonFX yeeterMotor = new TalonFX(55, ...);   // also in Spindexer!
 *
 * Having two CTRE objects for the same CAN ID produces "duplicate device"
 * warnings in the Driver Station console and can cause config conflicts
 * when both objects try to apply configurations to the same physical motor.
 *
 * THE FIX: DashboardTelemetry now receives the Shooter and Spindexer
 * subsystems as constructor parameters and calls getter methods on them
 * to read current draw. The motors themselves live only in their own
 * subsystems — this class just asks "how much current are you drawing?"
 *
 * This is like asking a colleague for a status update instead of
 * creating a second copy of their entire job description.
 * =====================================================================
 */
public class DashboardTelemetry extends SubsystemBase {

    // =========================================================
    // SUBSYSTEM REFERENCES
    // We read from these — we do not control them.
    // =========================================================
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter   shooter;
    private final Spindexer spindexer;

    // NetworkTables table for all test/debug data
    private final NetworkTable table = NetworkTableInstance
        .getDefault().getTable("Testing");

    // ---- POSE publishers ----
    private final DoublePublisher pigeonYawDeg =
        table.getDoubleTopic("Drive/PigeonYawDegrees").publish();
    private final DoublePublisher fusedYawDeg =
        table.getDoubleTopic("Drive/FusedYawDegrees").publish();

    // ---- TARGETING publishers ----
    private final DoublePublisher distanceMeters =
        table.getDoubleTopic("Targeting/DistanceToHubMeters").publish();
    private final DoublePublisher distanceFeet =
        table.getDoubleTopic("Targeting/DistanceToHubFeet").publish();
    private final DoublePublisher shooterTargetRPS =
        table.getDoubleTopic("Targeting/ShooterTargetRPS").publish();

    // ---- TEMPERATURE publishers ----
    private final DoublePublisher llATempF =
        table.getDoubleTopic("Temp/LimelightA_F").publish();
    private final DoublePublisher llBTempF =
        table.getDoubleTopic("Temp/LimelightB_F").publish();

    // ---- POWER publishers ----
    private final DoublePublisher shooterAmps =
        table.getDoubleTopic("Power/ShooterAmps").publish();
    private final DoublePublisher spindexerAmps =
        table.getDoubleTopic("Power/SpindexerAmps").publish();
    private final DoublePublisher yeeterAmps =
        table.getDoubleTopic("Power/YeeterAmps").publish();
    private final DoublePublisher swerveFlDriveAmps =
        table.getDoubleTopic("Power/SwerveFLDriveAmps").publish();
    private final DoublePublisher swerveFRDriveAmps =
        table.getDoubleTopic("Power/SwerveFRDriveAmps").publish();
    private final DoublePublisher swerveBLDriveAmps =
        table.getDoubleTopic("Power/SwerveBLDriveAmps").publish();
    private final DoublePublisher swerveBRDriveAmps =
        table.getDoubleTopic("Power/SwerveBRDriveAmps").publish();
    private final DoublePublisher totalAmps =
        table.getDoubleTopic("Power/TotalAmps").publish();

    // Swerve drive motors — these are safe to duplicate because
    // DashboardTelemetry only ever READS from them (getStatorCurrent),
    // never writes configurations or control requests to them.
    // The drivetrain subsystem owns their motion control exclusively.
    private final TalonFX flDrive = new TalonFX(43, Constants.kCANBus.getName());
    private final TalonFX frDrive = new TalonFX(13, Constants.kCANBus.getName());
    private final TalonFX blDrive = new TalonFX(33, Constants.kCANBus.getName());
    private final TalonFX brDrive = new TalonFX(23, Constants.kCANBus.getName());

    // =========================================================
    // CONSTRUCTOR
    // =========================================================
    /**
     * @param drivetrain  The swerve drivetrain subsystem (for pose and swerve current)
     * @param shooter     The shooter subsystem (for flywheel current, via getter)
     * @param spindexer   The spindexer subsystem (for spindexer + yeeter current, via getters)
     */
    public DashboardTelemetry(
        CommandSwerveDrivetrain drivetrain,
        Shooter shooter,
        Spindexer spindexer
    ) {
        setName("DashboardTelemetry");
        this.drivetrain = drivetrain;
        this.shooter    = shooter;
        this.spindexer  = spindexer;

        // =====================================================================
        // NOTE: getShotLeadSeconds() has been REMOVED from this class.
        //
        // The old code had a dashboard-adjustable "shot lead time" (how long
        // to wait after spin-up before feeding). That concept no longer applies
        // because the shoot sequence now uses:
        //   Commands.waitUntil(shooter::isAtTargetSpeed).withTimeout(1.5)
        //
        // That waits for the flywheel to ACTUALLY reach target speed, which is
        // more reliable than any fixed timer. There is no "lead time" to tune.
        //
        // If you ever want to make the 1.5-second timeout configurable from
        // the dashboard, add a DoubleSubscriber here and use it in RobotContainer
        // inside the .withTimeout() call. But for now, 1.5s is a safe default
        // that does not need in-match adjustment.
        // =====================================================================
    }

    @Override
    public void periodic() {

        // ---- Distance to alliance hub ----
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        Translation2d hub = getAllianceHub();
        double distM = robotPos.getDistance(hub);
        distanceMeters.set(distM);
        distanceFeet.set(distM * 3.28084);
        shooterTargetRPS.set(Constants.SHOOTER_SPEED_MAP.get(distM));

        // ---- Heading ----
        // RawHeading = pure Pigeon gyro, no odometry or vision blended in.
        // Pose rotation = the fused estimate (gyro + wheels + cameras).
        // Comparing these two in Elastic tells you how much vision is shifting
        // the pose vs. what the gyro alone thinks.
        pigeonYawDeg.set(drivetrain.getState().RawHeading.getDegrees());
        fusedYawDeg.set(drivetrain.getState().Pose.getRotation().getDegrees());

        // ---- Limelight temperatures ----
        llATempF.set(getLimelightTempF("limelight-a"));
        llBTempF.set(getLimelightTempF("limelight-b"));

        // ---- Motor current draw ----
        // Stator current = how hard the motor is working (load current).
        // This is what you use for brownout prediction and thermal monitoring.
        //
        // Shooter and Spindexer current comes from getter methods on those
        // subsystems — no duplicate motor handles needed here.
        double sa  = shooter.getStatorCurrentAmps();
        double spa = spindexer.getSpindexerStatorCurrentAmps();
        double ya  = spindexer.getYeeterStatorCurrentAmps();
        double fl  = Math.abs(flDrive.getStatorCurrent().getValueAsDouble());
        double fr  = Math.abs(frDrive.getStatorCurrent().getValueAsDouble());
        double bl  = Math.abs(blDrive.getStatorCurrent().getValueAsDouble());
        double br  = Math.abs(brDrive.getStatorCurrent().getValueAsDouble());

        shooterAmps.set(sa);
        spindexerAmps.set(spa);
        yeeterAmps.set(ya);
        swerveFlDriveAmps.set(fl);
        swerveFRDriveAmps.set(fr);
        swerveBLDriveAmps.set(bl);
        swerveBRDriveAmps.set(br);
        totalAmps.set(sa + spa + ya + fl + fr + bl + br);
    }

    // ---- Alliance hub selection ----
    private Translation2d getAllianceHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            return Constants.RED_HUB;
        return Constants.BLUE_HUB;
    }

    // ---- Limelight CPU temperature ----
    // Reads the 'hw' JSON entry that Limelight publishes to NetworkTables.
    // Returns temperature in Fahrenheit, or -1 if the camera is unavailable.
    // JSON looks like: {"cpu0temp":45.2,...}
    private double getLimelightTempF(String limelightName) {
        try {
            var llTable = NetworkTableInstance.getDefault().getTable(limelightName);
            String hwJson = llTable.getEntry("hw").getString("");
            if (hwJson.isEmpty()) return -1.0;
            int idx   = hwJson.indexOf("cpu0temp");
            if (idx < 0) return -1.0;
            int colon = hwJson.indexOf(':', idx);
            int end   = hwJson.indexOf(',', colon);
            if (end < 0) end = hwJson.indexOf('}', colon);
            double tempC = Double.parseDouble(hwJson.substring(colon + 1, end).trim());
            return (tempC * 9.0 / 5.0) + 32.0;
        } catch (Exception e) {
            return -1.0;
        }
    }
}