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
    // private final TalonFX leftMotor  = new TalonFX(60, Constants.kCANBus.getName());
    private final TalonFX rightMotor = new TalonFX(61, Constants.kCANBus.getName());

    // ===== CONTROL REQUEST =====
    // VelocityVoltage = cruise control for the flywheel.
    // We command a target RPS and the motor holds it automatically.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    // ===== DEFAULT STARTING GAINS =====
    // These are just safe starting points — you'll tune them live
    // from your dashboard without touching this file.
    //
    // kV: feedforward — your "good first guess" at how much voltage
    //     is needed per RPS. Formula: 12V ÷ ~100 RPS free speed = 0.12
    // kS: static friction — small voltage to overcome motor stiction
    // kP: proportional — corrects leftover error after feedforward
    //
    // TUNING ORDER: kV first → kS second → kP last
    private static final double DEFAULT_kV = 0.12;
    private static final double DEFAULT_kS = 0.0;
    private static final double DEFAULT_kP = 0.3;

    // ===== DEFAULT TARGET SPEED =====
    // Starting point for tuning. Adjust live from the dashboard.
    private static final double DEFAULT_TARGET_RPS = 65.0;

    // ===== NETWORKTABLES TABLE =====
    // All shooter-related NT entries live under "Shooter/"
    // Think of this as a folder that holds all our shooter values.
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault()
        .getTable("Shooter");

    // ===== TUNING INPUTS (writable from dashboard) =====
    // These are "subscribers" — the dashboard writes to them,
    // and our code reads them every loop.
    //
    // The second argument is the DEFAULT value used if the dashboard
    // hasn't written anything yet.
    private final DoubleSubscriber kVSub = shooterTable
        .getDoubleTopic("Tuning/kV").subscribe(DEFAULT_kV);
    private final DoubleSubscriber kSSub = shooterTable
        .getDoubleTopic("Tuning/kS").subscribe(DEFAULT_kS);
    private final DoubleSubscriber kPSub = shooterTable
        .getDoubleTopic("Tuning/kP").subscribe(DEFAULT_kP);
    private final DoubleSubscriber targetRPSSub = shooterTable
        .getDoubleTopic("Tuning/TargetRPS").subscribe(DEFAULT_TARGET_RPS);

    // ===== TELEMETRY OUTPUTS (read-only in dashboard) =====
    // These are "publishers" — our code writes to them,
    // and the dashboard just reads/displays them.
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
    // re-apply them when something actually changes.
    // Think of it like only calling your accountant when your
    // finances change — not every single day.
    private double lastKV = -1;
    private double lastKS = -1;
    private double lastKP = -1;

    // ===== RUNNING FLAG =====
    // Tracks whether the shooter is currently supposed to be spinning.
    // We need this so periodic() can keep sending the right target RPS
    // if the user adjusts the target speed live while shooting.
    private boolean isRunning = false;

    // ===== CONSTRUCTOR =====
    public Shooter() {
        setName("Shooter");

        var rightConfig = new TalonFXConfiguration();

        // Invert so both motors push the ball the same direction
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Apply starting gains — these will get overwritten the first
        // time periodic() detects a change from the dashboard
        rightConfig.Slot0.kV = DEFAULT_kV;
        rightConfig.Slot0.kS = DEFAULT_kS;
        rightConfig.Slot0.kP = DEFAULT_kP;
        rightConfig.Slot0.kI = 0.0;
        rightConfig.Slot0.kD = 0.0;

        rightMotor.getConfigurator().apply(rightConfig);

        // Set the default values on the dashboard so the user
        // sees something when they open it for the first time,
        // rather than a blank entry.
        shooterTable.getDoubleTopic("Tuning/kV").publish().set(DEFAULT_kV);
        shooterTable.getDoubleTopic("Tuning/kS").publish().set(DEFAULT_kS);
        shooterTable.getDoubleTopic("Tuning/kP").publish().set(DEFAULT_kP);
        shooterTable.getDoubleTopic("Tuning/TargetRPS").publish().set(DEFAULT_TARGET_RPS);
    }

    // ===== METHODS =====

    /**
     * Rev up the shooter to the target RPS set on the dashboard.
     * The motor holds that speed closed-loop even as battery drains.
     */
    public void runShooter() {
        isRunning = true;
        double targetRPS = targetRPSSub.get();
        // leftMotor.setControl(velocityRequest.withVelocity(targetRPS));
        rightMotor.setControl(velocityRequest.withVelocity(targetRPS));
    }

    /**
     * Run the shooter at a specific RPS (for distance-based shooting).
     * NOTE: Constants.SHOOTER_SPEED_MAP must return RPS values, not 0-1.
     *
     * @param speedRPS Target flywheel speed in Rotations Per Second
     */
    public void runShooterAtSpeed(double speedRPS) {
        isRunning = true;
        speedRPS = Math.max(0.0, Math.min(100.0, speedRPS));
        // leftMotor.setControl(velocityRequest.withVelocity(speedRPS));
        rightMotor.setControl(velocityRequest.withVelocity(speedRPS));
    }

    /** Stop the shooter wheels */
    public void stopShooter() {
        isRunning = false;
        // leftMotor.setControl(velocityRequest.withVelocity(0.0));
        rightMotor.setControl(velocityRequest.withVelocity(0.0));
    }

    // ===== PERIODIC =====
    @Override
    public void periodic() {

        // ---- STEP 1: Check if gains changed, re-apply if so ----
        // Read the current values from the dashboard
        double currentKV = kVSub.get();
        double currentKS = kSSub.get();
        double currentKP = kPSub.get();

        // Only send a config update to the motor if something actually changed.
        // Applying configs is an expensive CAN operation — we don't want to
        // spam it 50 times per second.
        if (currentKV != lastKV || currentKS != lastKS || currentKP != lastKP) {
            Slot0Configs newGains = new Slot0Configs();
            newGains.kV = currentKV;
            newGains.kS = currentKS;
            newGains.kP = currentKP;
            newGains.kI = 0.0;
            newGains.kD = 0.0;
            rightMotor.getConfigurator().apply(newGains);
            // leftMotor.getConfigurator().apply(newGains);

            // Remember what we just applied
            lastKV = currentKV;
            lastKS = currentKS;
            lastKP = currentKP;

            // Publish the active gains so you can confirm in the dashboard
            // that the motor actually received your new values
            activeKVPub.set(currentKV);
            activeKSPub.set(currentKS);
            activeKPPub.set(currentKP);
        }

        // ---- STEP 2: If running, keep sending the current target RPS ----
        // This is important — if you adjust TargetRPS on the dashboard
        // while the shooter is already spinning, this makes it take effect
        // immediately without stopping and restarting the shooter.
        if (isRunning) {
            double targetRPS = targetRPSSub.get();
            rightMotor.setControl(velocityRequest.withVelocity(targetRPS));
        }

        // ---- STEP 3: Publish telemetry ----
        double actualRPS = rightMotor.getVelocity().getValueAsDouble();
        double targetRPS = targetRPSSub.get();

        actualRPSPub.set(actualRPS);
        motorVoltsPub.set(rightMotor.getMotorVoltage().getValueAsDouble());

        // RPS Error = how far off we are from the target.
        // While tuning, watch this in AdvantageScope — you want it
        // to settle close to zero quickly with minimal oscillation.
        rpsErrorPub.set(targetRPS - actualRPS);
    }
}