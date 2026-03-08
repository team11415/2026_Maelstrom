package frc.robot.subsystems;

import frc.robot.Constants;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Shooter extends SubsystemBase {

    // ===== HARDWARE =====
    // private final TalonFX leftMotor = new TalonFX(60, Constants.kCANBus.getName());
    private final TalonFX rightMotor = new TalonFX(61, Constants.kCANBus.getName());

    // ===== CONTROL REQUEST =====
    // VelocityVoltage = cruise control for the flywheel.
    // We command a target RPS and the motor holds it automatically.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    // ===== DEFAULT STARTING GAINS =====
    // These are safe starting points — tune them live from the dashboard.
    //
    // kV: feedforward — first-guess at how much voltage per RPS.
    //     Formula: 12V ÷ ~100 RPS free speed = 0.12
    // kS: static friction — small voltage to overcome motor stiction
    // kP: proportional — corrects leftover error after feedforward
    //
    // TUNING ORDER: kV first → kS second → kP last
    private static final double DEFAULT_kV = 0.12;
    private static final double DEFAULT_kS = 0.0;
    private static final double DEFAULT_kP = 0.3;

    // ===== DEFAULT TARGET SPEED =====
    // Starting point for manual (non-distance-based) shooting.
    // Adjust live from the dashboard.
    private static final double DEFAULT_TARGET_RPS = 65.0;

    // ===== NETWORKTABLES TABLE =====
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault()
        .getTable("Shooter");

    // ===== TUNING INPUTS (writable from dashboard) =====
    private final DoubleSubscriber kVSub = shooterTable
        .getDoubleTopic("Tuning/kV").subscribe(DEFAULT_kV);
    private final DoubleSubscriber kSSub = shooterTable
        .getDoubleTopic("Tuning/kS").subscribe(DEFAULT_kS);
    private final DoubleSubscriber kPSub = shooterTable
        .getDoubleTopic("Tuning/kP").subscribe(DEFAULT_kP);
    private final DoubleSubscriber targetRPSSub = shooterTable
        .getDoubleTopic("Tuning/TargetRPS").subscribe(DEFAULT_TARGET_RPS);

    // ===== TELEMETRY OUTPUTS (read-only in dashboard) =====
    private final DoublePublisher actualRPSPub = shooterTable
        .getDoubleTopic("ActualRPS").publish();
    private final DoublePublisher motorVoltsPub = shooterTable
        .getDoubleTopic("MotorVolts").publish();
    private final DoublePublisher rpsErrorPub = shooterTable
        .getDoubleTopic("RPS_Error").publish();
    private final DoublePublisher activeKVPub = shooterTable
        .getDoubleTopic("Active/kV").publish();
    private final DoublePublisher activeKSPub = shooterTable
        .getDoubleTopic("Active/kS").publish();
    private final DoublePublisher activeKPPub = shooterTable
        .getDoubleTopic("Active/kP").publish();

    // ===== GAIN TRACKING =====
    // We remember the last gains we sent to the motor so we only
    // re-apply them when something actually changes — applying configs
    // is an expensive CAN operation and we don't want to spam it.
    private double lastKV = -1;
    private double lastKS = -1;
    private double lastKP = -1;

    // ===== RUNNING FLAG =====
    private boolean isRunning = false;

    // ===== ACTIVE TARGET RPS =====
    // -----------------------------------------------------------------------
    // THE FIX: This field stores whatever speed was most recently commanded,
    // whether that came from runShooter() (dashboard value) or
    // runShooterAtSpeed() (distance-based calculation).
    //
    // WHY THIS MATTERS:
    //   periodic() runs 50 times per second. The OLD code always read
    //   targetRPSSub.get() (the dashboard value) inside periodic(). That
    //   meant if runShooterAtSpeed(75.0) was called to shoot from far away,
    //   periodic() would silently overwrite it with the dashboard value
    //   (e.g. 65.0) just 20ms later. The distance-based shot speed was
    //   effectively ignored after the very first loop.
    //
    // THE FIX:
    //   Both runShooter() and runShooterAtSpeed() now store their chosen
    //   speed into activeTargetRPS. periodic() re-commands THAT stored value
    //   instead of re-reading the dashboard subscriber directly.
    //
    //   Think of it like a sticky note: whoever calls run*() writes the speed
    //   they want on the note. periodic() just keeps re-sending whatever is
    //   written on the note — it never writes on the note itself.
    // -----------------------------------------------------------------------
    private double activeTargetRPS = DEFAULT_TARGET_RPS;

    // ===== CONSTRUCTOR =====
    public Shooter() {
        setName("Shooter");

        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.Slot0.kV = DEFAULT_kV;
        rightConfig.Slot0.kS = DEFAULT_kS;
        rightConfig.Slot0.kP = DEFAULT_kP;
        rightConfig.Slot0.kI = 0.0;
        rightConfig.Slot0.kD = 0.0;
        rightMotor.getConfigurator().apply(rightConfig);

        // Publish defaults to dashboard so widgets aren't blank on first open
        shooterTable.getDoubleTopic("Tuning/kV").publish().set(DEFAULT_kV);
        shooterTable.getDoubleTopic("Tuning/kS").publish().set(DEFAULT_kS);
        shooterTable.getDoubleTopic("Tuning/kP").publish().set(DEFAULT_kP);
        shooterTable.getDoubleTopic("Tuning/TargetRPS").publish().set(DEFAULT_TARGET_RPS);
    }

    // ===== METHODS =====

    /**
     * Rev up the shooter to the target RPS set on the dashboard.
     *
     * This is used for manual shooting (e.g., from a fixed known distance
     * or when you just want a quick test spin).
     *
     * The dashboard value is captured into activeTargetRPS at the moment
     * this is called. periodic() will keep re-commanding that captured value.
     * If you adjust the dashboard while shooting, the new value takes effect
     * on the NEXT call to runShooter() — not instantly mid-shot.
     * (This is intentional — you don't want speed changing while the ball
     * is being fed.)
     */
    public void runShooter() {
        isRunning = true;
        // Capture the current dashboard value into activeTargetRPS.
        // periodic() will re-use this stored value each loop.
        activeTargetRPS = targetRPSSub.get();
        // leftMotor.setControl(velocityRequest.withVelocity(activeTargetRPS));
        rightMotor.setControl(velocityRequest.withVelocity(activeTargetRPS));
    }

    /**
     * Run the shooter at a specific RPS calculated from shooting distance.
     *
     * Called by the right-trigger shoot command in RobotContainer, which
     * looks up the correct speed from Constants.SHOOTER_SPEED_MAP based on
     * how far the robot is from the target.
     *
     * The computed speed is stored in activeTargetRPS. periodic() will
     * keep re-commanding this exact value each loop — it will NOT fall back
     * to the dashboard's TargetRPS, so the distance-based speed is preserved
     * for the entire duration of the shot.
     *
     * @param speedRPS Target flywheel speed in Rotations Per Second.
     *                 Clamped to [0, 100] as a safety guard.
     */
    public void runShooterAtSpeed(double speedRPS) {
        isRunning = true;
        // Clamp to a safe range — Kraken x60 free-spins at ~100 RPS.
        // This prevents accidental commands like 500 RPS from a bad lookup.
        activeTargetRPS = Math.max(0.0, Math.min(100.0, speedRPS));
        // leftMotor.setControl(velocityRequest.withVelocity(activeTargetRPS));
        rightMotor.setControl(velocityRequest.withVelocity(activeTargetRPS));
    }

    /** Stop the shooter wheels. Clears the running flag and zeros the stored speed. */
    public void stopShooter() {
        isRunning = false;
        activeTargetRPS = 0.0; // clear the stored speed so it doesn't linger
        // leftMotor.setControl(velocityRequest.withVelocity(0.0));
        rightMotor.setControl(velocityRequest.withVelocity(0.0));
    }

    // ===== PERIODIC =====
    @Override
    public void periodic() {

        // ---- STEP 1: Check if gains changed, re-apply if so ----
        // Only send a config update to the motor if something actually changed.
        double currentKV = kVSub.get();
        double currentKS = kSSub.get();
        double currentKP = kPSub.get();

        if (currentKV != lastKV || currentKS != lastKS || currentKP != lastKP) {
            Slot0Configs newGains = new Slot0Configs();
            newGains.kV = currentKV;
            newGains.kS = currentKS;
            newGains.kP = currentKP;
            newGains.kI = 0.0;
            newGains.kD = 0.0;
            rightMotor.getConfigurator().apply(newGains);
            // leftMotor.getConfigurator().apply(newGains);

            lastKV = currentKV;
            lastKS = currentKS;
            lastKP = currentKP;

            activeKVPub.set(currentKV);
            activeKSPub.set(currentKS);
            activeKPPub.set(currentKP);
        }

        // ---- STEP 2: If running, keep commanding the STORED active target ----
        // KEY CHANGE from the old code:
        //   OLD: rightMotor.setControl(velocityRequest.withVelocity(targetRPSSub.get()))
        //        ↑ This always read the dashboard value, overwriting any
        //          distance-based speed set by runShooterAtSpeed().
        //
        //   NEW: rightMotor.setControl(velocityRequest.withVelocity(activeTargetRPS))
        //        ↑ This re-commands whatever speed was last stored by
        //          runShooter() or runShooterAtSpeed() — dashboard or computed.
        if (isRunning) {
            // leftMotor.setControl(velocityRequest.withVelocity(activeTargetRPS));
            rightMotor.setControl(velocityRequest.withVelocity(activeTargetRPS));
        }

        // ---- STEP 3: Publish telemetry ----
        double actualRPS = rightMotor.getVelocity().getValueAsDouble();
        actualRPSPub.set(actualRPS);
        motorVoltsPub.set(rightMotor.getMotorVoltage().getValueAsDouble());
        // RPS Error = how far the motor is from the target speed.
        // Watch this in AdvantageScope while tuning — you want it to
        // settle close to zero quickly with minimal oscillation.
        rpsErrorPub.set(activeTargetRPS - actualRPS);
    }
}