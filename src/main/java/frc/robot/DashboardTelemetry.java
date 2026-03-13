package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

// This subsystem publishes extra telemetry for testing.
// It doesn't control anything — just monitors and reports.
public class DashboardTelemetry extends SubsystemBase {

    // References to other subsystems (we read from them, don't control them)
    private final CommandSwerveDrivetrain drivetrain;

    // NetworkTables table for all our test/debug data
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

    // ---- SHOT LEAD TIME (writable from dashboard) ----
    // Default 0.25 seconds — time between flywheel spin-up and ball feed.
    // This is readable via getShotLeadSeconds() if other code needs it.
    private final DoubleSubscriber shotLeadSub =
        table.getDoubleTopic("Targeting/ShotLeadSeconds").subscribe(0.25);

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

    // The swerve drive/steer motors (IDs from Constants.java)
    // Drive: FL=43, FR=13, BL=33, BR=23
    private final TalonFX flDrive = new TalonFX(43, Constants.kCANBus.getName());
    private final TalonFX frDrive = new TalonFX(13, Constants.kCANBus.getName());
    private final TalonFX blDrive = new TalonFX(33, Constants.kCANBus.getName());
    private final TalonFX brDrive = new TalonFX(23, Constants.kCANBus.getName());
    private final TalonFX shooterRight = new TalonFX(61, Constants.kCANBus.getName());
    private final TalonFXS spindexerMotor = new TalonFXS(54, Constants.kCANBus.getName());
    private final TalonFX yeeterMotor = new TalonFX(55, Constants.kCANBus.getName());

    public DashboardTelemetry(CommandSwerveDrivetrain drivetrain) {
        setName("DashboardTelemetry");
        this.drivetrain = drivetrain;

        // =====================================================================
        // BUG FIX: Anonymous Publisher Garbage Collection
        // =====================================================================
        // OLD (broken) code:
        //   table.getDoubleTopic("Targeting/ShotLeadSeconds").publish().set(0.25);
        //
        // The problem: .publish() creates a Publisher object that was never
        // saved to a variable. Java's garbage collector can delete it almost
        // immediately, before the value ever reaches NetworkTables — so the
        // Elastic widget for shot lead time showed blank on first open.
        //
        // THE FIX: Use getEntry().setDefaultDouble() instead.
        //   - getEntry() returns a persistent reference that won't be collected.
        //   - setDefaultDouble() only writes if the key doesn't already exist,
        //     so it won't overwrite a value that was previously set by the user.
        // =====================================================================
        table.getEntry("Targeting/ShotLeadSeconds").setDefaultDouble(0.25);
    }

    /** Call this from RobotContainer to read the shot lead time. */
    public double getShotLeadSeconds() {
        return shotLeadSub.get();
    }

    @Override
    public void periodic() {
        // --- Distance to hub ---
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        Translation2d hub = getAllianceHub();
        double distM = robotPos.getDistance(hub);
        distanceMeters.set(distM);
        distanceFeet.set(distM * 3.28084);
        shooterTargetRPS.set(Constants.SHOOTER_SPEED_MAP.get(distM));

        // state.RawHeading is the Pigeon's own rotation, before pose fusion.
        // This is the "pure gyro" value — no odometry or vision mixed in.
        pigeonYawDeg.set(drivetrain.getState().RawHeading.getDegrees());
        fusedYawDeg.set(drivetrain.getState().Pose.getRotation().getDegrees());

        // --- Limelight temperatures (Celsius from Limelight, convert to F) ---
        llATempF.set(getLimelightTempF("limelight-a"));
        llBTempF.set(getLimelightTempF("limelight-b"));

        // --- Motor current draw (stator current = how hard motor is working) ---
        double sa = Math.abs(shooterRight.getStatorCurrent().getValueAsDouble());
        double spa = Math.abs(spindexerMotor.getStatorCurrent().getValueAsDouble());
        double ya = Math.abs(yeeterMotor.getStatorCurrent().getValueAsDouble());
        double fl = Math.abs(flDrive.getStatorCurrent().getValueAsDouble());
        double fr = Math.abs(frDrive.getStatorCurrent().getValueAsDouble());
        double bl = Math.abs(blDrive.getStatorCurrent().getValueAsDouble());
        double br = Math.abs(brDrive.getStatorCurrent().getValueAsDouble());
        shooterAmps.set(sa);
        spindexerAmps.set(spa);
        yeeterAmps.set(ya);
        swerveFlDriveAmps.set(fl);
        swerveFRDriveAmps.set(fr);
        swerveBLDriveAmps.set(bl);
        swerveBRDriveAmps.set(br);
        totalAmps.set(sa + spa + ya + fl + fr + bl + br);
    }

    private Translation2d getAllianceHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            return Constants.RED_HUB;
        return Constants.BLUE_HUB;
    }

    // Reads Limelight temperature from its NetworkTables 'hw' JSON blob.
    // Returns temperature in Fahrenheit, or -1 if the camera isn't available.
    private double getLimelightTempF(String limelightName) {
        try {
            var limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
            String hwJson = limelightTable.getEntry("hw").getString("");
            if (hwJson.isEmpty()) return -1.0;
            // Parse: find 'cpu0temp' value in JSON string
            // JSON looks like: {"cpu0temp":45.2,...}
            int idx = hwJson.indexOf("cpu0temp");
            if (idx < 0) return -1.0;
            int colon = hwJson.indexOf(':', idx);
            int end = hwJson.indexOf(',', colon);
            if (end < 0) end = hwJson.indexOf('}', colon);
            double tempC = Double.parseDouble(hwJson.substring(colon + 1, end).trim());
            // Convert Celsius to Fahrenheit: F = (C * 9/5) + 32
            return (tempC * 9.0 / 5.0) + 32.0;
        } catch (Exception e) {
            return -1.0; // Camera not available
        }
    }
}