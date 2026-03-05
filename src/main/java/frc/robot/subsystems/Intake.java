package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/**
 * Intake subsystem for Maelstrom 2026.
 *
 * ============================================================
 * HARDWARE OVERVIEW:
 * ============================================================
 *
 *  LEFT EXTEND MOTOR:   TalonFXS, CAN ID 50, Minion (Minion_JST)
 *  RIGHT EXTEND MOTOR:  TalonFXS, CAN ID 51, Minion (Minion_JST) — INVERTED
 *  ROLLER MOTOR:        TalonFX,  CAN ID 52, Kraken x44
 *
 *  Each TalonFXS has TWO hardware limit switches wired to it:
 *
 *    Forward limit  = intake is FULLY EXTENDED (deployed out of robot)
 *    Reverse limit  = intake is FULLY RETRACTED (tucked inside robot)
 *
 *  Think of the limit switches like the end-stops on a sliding drawer:
 *  one bumper at the back says "stop, you're in", one at the front says
 *  "stop, you're fully out". The motor controller enforces these
 *  automatically in hardware — no software check needed.
 *
 * ============================================================
 * HOW MOTIONMAGIC WORKS (simple explanation):
 * ============================================================
 *
 *  Normal position control just slams to the target and hopes for the best.
 *  MotionMagic is like cruise control: it smoothly ramps up to a cruise
 *  speed, holds it, then smoothly decelerates to stop at the target position.
 *
 *  You tune three things:
 *    cruiseVelocity  = max speed during the move (rot/sec)
 *    acceleration    = how quickly it reaches cruise speed (rot/sec²)
 *    jerk            = how smoothly it enters/exits acceleration (rot/sec³)
 *                      (set to 0 to disable jerk limiting)
 *
 *  Plus the usual Slot0 gains:
 *    kV  = feedforward voltage per RPS (first-guess power)
 *    kS  = static friction offset (helps motor get moving)
 *    kP  = proportional error correction (fixes leftover position error)
 *    kD  = derivative damping (reduces oscillation/overshoot)
 *
 * ============================================================
 * POSITION SYSTEM:
 * ============================================================
 *
 *  The reverse limit switch (retracted) automatically zeros the encoder
 *  every time the intake retracts fully. So:
 *
 *    0.0 rotations  = fully retracted (reverse limit triggered)
 *    EXTENDED_ROTATIONS = fully extended (forward limit triggered)
 *
 *  EXTENDED_ROTATIONS is how many motor shaft rotations it takes to
 *  go from fully retracted to fully extended. Measure this on the
 *  real robot: retract fully, then manually extend and read the
 *  encoder value in Phoenix Tuner X. Set EXTENDED_ROTATIONS to that value.
 *
 * ============================================================
 * USAGE (from RobotContainer.java):
 * ============================================================
 *
 *  Intake intake = new Intake();
 *
 *  // Left trigger held → extend and spin:
 *  driver.leftTrigger(0.5).whileTrue(intake.extendAndRunCommand());
 *
 *  // Robot will automatically retract when trigger is released because
 *  // extendAndRunCommand() calls retract() + stopRoller() on end.
 */
public class Intake extends SubsystemBase {

    // =========================================================
    // HARDWARE
    // =========================================================

    // Left extend motor — TalonFXS (note the S), Minion motor
    private final TalonFXS leftExtend  = new TalonFXS(50, Constants.kCANBus.getName());

    // Right extend motor — TalonFXS, Minion motor, INVERTED
    // Because the right motor is on the opposite side of the intake,
    // "clockwise" on the right is "counterclockwise" on the left.
    // Inverting makes both motors push the intake in the same direction.
    private final TalonFXS rightExtend = new TalonFXS(51, Constants.kCANBus.getName());

    // Roller motor — TalonFX (standard), Kraken x44
    // This just spins to pull game pieces into the robot.
    private final TalonFX  roller      = new TalonFX(52,  Constants.kCANBus.getName());


    // =========================================================
    // CONTROL REQUESTS
    // =========================================================

    // MotionMagicVoltage: command a position (in rotations) and let
    // the motor smoothly ramp up and down to reach it.
    // withSlot(0) means "use the gains in Slot 0" (the only slot we configure).
    private final MotionMagicVoltage extendRequest =
        new MotionMagicVoltage(0.0).withSlot(0);

    // VelocityVoltage: command a target RPS for the roller.
    // Same pattern as the shooter and spindexer in this codebase.
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0.0);


    // =========================================================
    // POSITION CONSTANTS
    // =========================================================

    // Retracted position is always 0 because the reverse limit switch
    // automatically zeros the encoder every time the intake retracts fully.
    private static final double RETRACTED_ROTATIONS = 0.0;

    // >>> MEASURE THIS ON THE REAL ROBOT <<<
    // Steps to find this value:
    //   1. Deploy code. Let the intake retract fully (reverse limit zeroes encoder).
    //   2. Manually command the motor forward slowly until the intake is fully deployed.
    //   3. Open Phoenix Tuner X → select motor 50 → look at "Position" in the Signal tab.
    //   4. Copy that number here.
    // A good starting guess is somewhere between 3.0 and 15.0 rotations
    // depending on your gearing.
    private static final double EXTENDED_ROTATIONS = 8.0; // >>> MEASURE THIS <<<


    // =========================================================
    // MOTIONMAGIC TUNING DEFAULTS (extend/retract motors)
    // =========================================================

    // How fast the intake moves during extension/retraction (rotations/sec)
    // Start slow (2.0) and increase once you've verified motion is smooth.
    private static final double DEFAULT_MM_CRUISE_VEL  = 2.0;   // rot/sec

    // How quickly it accelerates to cruise velocity (rotations/sec²)
    // Lower = gentler, higher = snappier
    private static final double DEFAULT_MM_ACCEL       = 4.0;   // rot/sec²

    // How smoothly it transitions into/out of acceleration (rotations/sec³)
    // 0 = disabled (sharp transitions), higher = S-curve smoothing
    private static final double DEFAULT_MM_JERK        = 0.0;   // rot/sec³

    // Slot0 gains for the extend motors
    // kV: first-guess feedforward. Minion free-speed is ~100 RPS.
    //     12V / 100 RPS = 0.12 is a safe starting point.
    // kS: static friction offset. Start at 0, add in 0.01 steps if motor hesitates.
    // kP: proportional correction. Tune after kV/kS are set.
    // kD: derivative damping. Helps reduce oscillation/overshoot at target position.
    private static final double DEFAULT_EXTEND_kV = 0.12;
    private static final double DEFAULT_EXTEND_kS = 0.0;
    private static final double DEFAULT_EXTEND_kP = 1.0;  // Starting guess — tune up/down
    private static final double DEFAULT_EXTEND_kD = 0.0;  // Add if position overshoots


    // =========================================================
    // ROLLER TUNING DEFAULTS
    // =========================================================

    // Target roller speed in RPS. Kraken x44 free-spins at ~100 RPS.
    // Tune this to intake reliably without jamming.
    private static final double DEFAULT_ROLLER_TARGET_RPS = 40.0;  // Start slow, increase
    private static final double DEFAULT_ROLLER_kV         = 0.12;  // 12V / ~100 RPS
    private static final double DEFAULT_ROLLER_kS         = 0.0;
    private static final double DEFAULT_ROLLER_kP         = 0.1;


    // =========================================================
    // NETWORKTABLES SETUP
    // =========================================================

    private final NetworkTable table = NetworkTableInstance.getDefault()
        .getTable("Intake");

    // ---- Tuning inputs (writable from Elastic dashboard) ----
    // MotionMagic parameters
    private final DoubleSubscriber mmCruiseSub = table
        .getDoubleTopic("Tuning/MM_CruiseVelocity").subscribe(DEFAULT_MM_CRUISE_VEL);
    private final DoubleSubscriber mmAccelSub  = table
        .getDoubleTopic("Tuning/MM_Acceleration").subscribe(DEFAULT_MM_ACCEL);
    private final DoubleSubscriber mmJerkSub   = table
        .getDoubleTopic("Tuning/MM_Jerk").subscribe(DEFAULT_MM_JERK);

    // Extend motor gains
    private final DoubleSubscriber extKVSub = table
        .getDoubleTopic("Tuning/Extend_kV").subscribe(DEFAULT_EXTEND_kV);
    private final DoubleSubscriber extKSSub = table
        .getDoubleTopic("Tuning/Extend_kS").subscribe(DEFAULT_EXTEND_kS);
    private final DoubleSubscriber extKPSub = table
        .getDoubleTopic("Tuning/Extend_kP").subscribe(DEFAULT_EXTEND_kP);
    private final DoubleSubscriber extKDSub = table
        .getDoubleTopic("Tuning/Extend_kD").subscribe(DEFAULT_EXTEND_kD);

    // Roller gains and speed
    private final DoubleSubscriber rollerRPSSub = table
        .getDoubleTopic("Tuning/Roller_TargetRPS").subscribe(DEFAULT_ROLLER_TARGET_RPS);
    private final DoubleSubscriber rollerKVSub  = table
        .getDoubleTopic("Tuning/Roller_kV").subscribe(DEFAULT_ROLLER_kV);
    private final DoubleSubscriber rollerKSSub  = table
        .getDoubleTopic("Tuning/Roller_kS").subscribe(DEFAULT_ROLLER_kS);
    private final DoubleSubscriber rollerKPSub  = table
        .getDoubleTopic("Tuning/Roller_kP").subscribe(DEFAULT_ROLLER_kP);

    // ---- Telemetry outputs (read-only in dashboard) ----
    private final DoublePublisher leftPosPub  = table
        .getDoubleTopic("LeftExtend_Position").publish();
    private final DoublePublisher rightPosPub = table
        .getDoubleTopic("RightExtend_Position").publish();
    private final DoublePublisher rollerRPSPub = table
        .getDoubleTopic("Roller_ActualRPS").publish();
    private final DoublePublisher rollerErrorPub = table
        .getDoubleTopic("Roller_RPS_Error").publish();

    // Are each side's limit switches triggered right now?
    private final BooleanPublisher leftFwdLimitPub  = table
        .getBooleanTopic("LeftExtend_AtExtended").publish();
    private final BooleanPublisher leftRevLimitPub  = table
        .getBooleanTopic("LeftExtend_AtRetracted").publish();
    private final BooleanPublisher rightFwdLimitPub = table
        .getBooleanTopic("RightExtend_AtExtended").publish();
    private final BooleanPublisher rightRevLimitPub = table
        .getBooleanTopic("RightExtend_AtRetracted").publish();

    // Is the intake currently commanded to be extended?
    private final BooleanPublisher deployedPub = table
        .getBooleanTopic("IsDeployed").publish();


    // =========================================================
    // GAIN TRACKING (to avoid spamming CAN with redundant configs)
    // =========================================================
    private double lastExtKV = -1, lastExtKS = -1, lastExtKP = -1, lastExtKD = -1;
    private double lastMmCruise = -1, lastMmAccel = -1, lastMmJerk = -1;
    private double lastRollerKV = -1, lastRollerKS = -1, lastRollerKP = -1;

    // =========================================================
    // STATE TRACKING
    // =========================================================
    private boolean isDeployed = false;  // Are we currently extended (or extending)?
    private boolean rollerRunning = false;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================
    public Intake() {
        setName("Intake");

        configureExtendMotor(leftExtend,  false); // Left motor: not inverted
        configureExtendMotor(rightExtend, true);  // Right motor: inverted

        configureRollerMotor();

        // Publish default tuning values to dashboard so widgets aren't blank
        publishDefaults();

        // START RETRACTED: Command both extend motors to retract immediately.
        // The reverse limit switch will stop them and zero the encoder.
        // This ensures the intake is safely inside the robot on boot.
        retract();
    }


    // =========================================================
    // MOTOR CONFIGURATION HELPERS
    // =========================================================

    /**
     * Configures one TalonFXS extend motor.
     *
     * Key things we set up:
     *   1. MotorArrangement = Minion_JST  (required for Minion motors on TalonFXS)
     *   2. Inversion direction
     *   3. Brake mode so the intake holds position when commanded to stop
     *   4. Hardware limit switches (forward = extended, reverse = retracted)
     *   5. Auto-zero encoder on reverse limit (so retracted is always 0)
     *   6. Starting Slot0 gains and MotionMagic parameters
     *
     * @param motor    The TalonFXS to configure
     * @param inverted Whether to invert this motor's direction
     */
    private void configureExtendMotor(TalonFXS motor, boolean inverted) {
        var config = new TalonFXSConfiguration();

        // ---- Motor type: REQUIRED for Minion motors on TalonFXS ----
        // Without this, the motor will not spin correctly or may not spin at all.
        // Think of it like telling the controller "this is a Minion, not a Falcon" —
        // it needs to know what kind of motor it's talking to.
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // ---- Direction ----
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        // ---- Brake mode ----
        // In brake mode, the motor actively resists being moved when it's
        // not being commanded. This holds the intake in position.
        // Coast mode would let it drift freely — we don't want that.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // ---- Hardware limit switches ----
        // These are wired physically to the TalonFXS limit switch pins.
        // NormallyOpen means the switch is OPEN (not triggered) in its
        // rest state, and CLOSES (triggers) when the intake hits the stop.
        // Change to NormallyClosed if your switches are wired the other way.
        //
        // IMPORTANT: ForwardLimitAutosetPositionEnable sets the encoder to
        // ForwardLimitAutosetPositionValue when the forward limit triggers.
        // We DON'T set forward auto-position because the physical forward
        // stop is more reliable.
        //
        // ReverseLimitAutosetPositionEnable zeros the encoder when the intake
        // retracts fully. This means "retracted = 0 rotations" is always true,
        // even if the motor lost its position during a match.
        var limitConfig = config.HardwareLimitSwitch;
        limitConfig.ForwardLimitEnable = true;
        limitConfig.ForwardLimitType   = ForwardLimitTypeValue.NormallyOpen;

        limitConfig.ReverseLimitEnable = true;
        limitConfig.ReverseLimitType   = ReverseLimitTypeValue.NormallyOpen;

        // Auto-zero when we hit the retracted (reverse) limit switch.
        // This is like a GPS that recalibrates every time you pull into your driveway.
        limitConfig.ReverseLimitAutosetPositionEnable = true;
        limitConfig.ReverseLimitAutosetPositionValue  = 0.0; // Retracted = 0 rotations

        // ---- Starting gains (will be overwritten from dashboard in periodic) ----
        config.Slot0.kV = DEFAULT_EXTEND_kV;
        config.Slot0.kS = DEFAULT_EXTEND_kS;
        config.Slot0.kP = DEFAULT_EXTEND_kP;
        config.Slot0.kD = DEFAULT_EXTEND_kD;
        config.Slot0.kI = 0.0; // Not needed with feedforward + kP + kD

        // ---- MotionMagic starting parameters ----
        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MM_CRUISE_VEL;
        config.MotionMagic.MotionMagicAcceleration   = DEFAULT_MM_ACCEL;
        config.MotionMagic.MotionMagicJerk           = DEFAULT_MM_JERK;

        motor.getConfigurator().apply(config);
    }

    /**
     * Configures the Kraken x44 roller motor (TalonFX, ID 52).
     * This just spins at a commanded RPS to pull game pieces in.
     */
    private void configureRollerMotor() {
        var config = new TalonFXConfiguration();

        // Adjust direction if the roller spins the wrong way — flip this value.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Coast mode for the roller: when we stop commanding it, let it spin down
        // naturally. Brake mode on a roller can cause it to "grab" a ball mid-feed.
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Starting gains
        config.Slot0.kV = DEFAULT_ROLLER_kV;
        config.Slot0.kS = DEFAULT_ROLLER_kS;
        config.Slot0.kP = DEFAULT_ROLLER_kP;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        roller.getConfigurator().apply(config);
    }

    /** Publishes all default values to the dashboard so widgets aren't blank on first open. */
    private void publishDefaults() {
        table.getDoubleTopic("Tuning/MM_CruiseVelocity").publish().set(DEFAULT_MM_CRUISE_VEL);
        table.getDoubleTopic("Tuning/MM_Acceleration").publish().set(DEFAULT_MM_ACCEL);
        table.getDoubleTopic("Tuning/MM_Jerk").publish().set(DEFAULT_MM_JERK);
        table.getDoubleTopic("Tuning/Extend_kV").publish().set(DEFAULT_EXTEND_kV);
        table.getDoubleTopic("Tuning/Extend_kS").publish().set(DEFAULT_EXTEND_kS);
        table.getDoubleTopic("Tuning/Extend_kP").publish().set(DEFAULT_EXTEND_kP);
        table.getDoubleTopic("Tuning/Extend_kD").publish().set(DEFAULT_EXTEND_kD);
        table.getDoubleTopic("Tuning/Roller_TargetRPS").publish().set(DEFAULT_ROLLER_TARGET_RPS);
        table.getDoubleTopic("Tuning/Roller_kV").publish().set(DEFAULT_ROLLER_kV);
        table.getDoubleTopic("Tuning/Roller_kS").publish().set(DEFAULT_ROLLER_kS);
        table.getDoubleTopic("Tuning/Roller_kP").publish().set(DEFAULT_ROLLER_kP);
    }


    // =========================================================
    // PUBLIC CONTROL METHODS
    // =========================================================

    /**
     * Commands the intake to extend to the deployed position.
     *
     * Both motors receive a MotionMagic position target of EXTENDED_ROTATIONS.
     * They will smoothly ramp up to cruise velocity, hold it, then decelerate
     * and stop at the target. The forward hardware limit switch will catch them
     * if they overshoot (it physically prevents further motion).
     */
    public void extend() {
        isDeployed = true;
        leftExtend.setControl(extendRequest.withPosition(EXTENDED_ROTATIONS));
        rightExtend.setControl(extendRequest.withPosition(EXTENDED_ROTATIONS));
    }

    /**
     * Commands the intake to retract to the fully-inside position (0 rotations).
     *
     * The reverse hardware limit switch stops the motors when fully retracted
     * and simultaneously zeros the encoder for the next extension cycle.
     */
    public void retract() {
        isDeployed = false;
        leftExtend.setControl(extendRequest.withPosition(RETRACTED_ROTATIONS));
        rightExtend.setControl(extendRequest.withPosition(RETRACTED_ROTATIONS));
    }

    /**
     * Spins the intake roller at the target RPS set on the dashboard.
     * Call this at the same time as extend().
     */
    public void runRoller() {
        rollerRunning = true;
        roller.setControl(rollerRequest.withVelocity(rollerRPSSub.get()));
    }

    /**
     * Stops the intake roller.
     * Call this when retracting.
     */
    public void stopRoller() {
        rollerRunning = false;
        roller.setControl(rollerRequest.withVelocity(0.0));
    }

    /**
     * Convenience method: extend the intake AND spin the roller at the same time.
     * Used as the "running" state of the left trigger command.
     */
    public void deployAndRun() {
        extend();
        runRoller();
    }

    /**
     * Convenience method: retract the intake AND stop the roller at the same time.
     * Used as the cleanup action when the left trigger is released.
     */
    public void retractAndStop() {
        retract();
        stopRoller();
    }

    /**
     * Returns true if the left extend motor's reverse limit switch is triggered,
     * meaning the intake is confirmed fully retracted.
     *
     * You can use this to interlock other mechanisms — e.g., don't allow
     * driving at full speed unless isFullyRetracted() is true.
     */
    public boolean isFullyRetracted() {
        return leftExtend.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Returns true if the left extend motor's forward limit switch is triggered,
     * meaning the intake is confirmed fully extended/deployed.
     */
    public boolean isFullyExtended() {
        return leftExtend.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }


    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {

        // ---- STEP 1: Check if extend gains changed, re-apply if needed ----
        // Applying configs is an expensive CAN bus operation — only do it when
        // values actually change, not every single loop.
        double kV = extKVSub.get(), kS = extKSSub.get(),
               kP = extKPSub.get(), kD = extKDSub.get();
        double cruise = mmCruiseSub.get(), accel = mmAccelSub.get(), jerk = mmJerkSub.get();

        boolean gainsChanged = (kV != lastExtKV || kS != lastExtKS ||
                                kP != lastExtKP || kD != lastExtKD);
        boolean mmChanged    = (cruise != lastMmCruise || accel != lastMmAccel ||
                                jerk != lastMmJerk);

        if (gainsChanged || mmChanged) {
            // Build an update with only the slots/configs we're changing.
            // This is more efficient than re-sending the entire configuration.
            var slot0 = new Slot0Configs();
            slot0.kV = kV; slot0.kS = kS; slot0.kP = kP; slot0.kD = kD; slot0.kI = 0.0;

            var mm = new MotionMagicConfigs();
            mm.MotionMagicCruiseVelocity = cruise;
            mm.MotionMagicAcceleration   = accel;
            mm.MotionMagicJerk           = jerk;

            // Apply to both extend motors so they stay in sync
            leftExtend.getConfigurator().apply(slot0);
            leftExtend.getConfigurator().apply(mm);
            rightExtend.getConfigurator().apply(slot0);
            rightExtend.getConfigurator().apply(mm);

            lastExtKV = kV; lastExtKS = kS; lastExtKP = kP; lastExtKD = kD;
            lastMmCruise = cruise; lastMmAccel = accel; lastMmJerk = jerk;
        }

        // ---- STEP 2: Check if roller gains changed, re-apply if needed ----
        double rKV = rollerKVSub.get(), rKS = rollerKSSub.get(), rKP = rollerKPSub.get();
        if (rKV != lastRollerKV || rKS != lastRollerKS || rKP != lastRollerKP) {
            var slot0 = new Slot0Configs();
            slot0.kV = rKV; slot0.kS = rKS; slot0.kP = rKP; slot0.kI = 0.0; slot0.kD = 0.0;
            roller.getConfigurator().apply(slot0);
            lastRollerKV = rKV; lastRollerKS = rKS; lastRollerKP = rKP;
        }

        // ---- STEP 3: If roller is running, keep feeding the latest target RPS ----
        // This means if you adjust Roller_TargetRPS on the dashboard while
        // the roller is already spinning, it takes effect immediately.
        if (rollerRunning) {
            roller.setControl(rollerRequest.withVelocity(rollerRPSSub.get()));
        }

        // ---- STEP 4: Publish telemetry ----
        leftPosPub.set(leftExtend.getPosition().getValueAsDouble());
        rightPosPub.set(rightExtend.getPosition().getValueAsDouble());

        double actualRPS = roller.getVelocity().getValueAsDouble();
        rollerRPSPub.set(actualRPS);
        rollerErrorPub.set(rollerRPSSub.get() - actualRPS);

        // Limit switch states (true = triggered = at this position)
        leftFwdLimitPub.set(
            leftExtend.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
        leftRevLimitPub.set(
            leftExtend.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
        rightFwdLimitPub.set(
            rightExtend.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
        rightRevLimitPub.set(
            rightExtend.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);

        deployedPub.set(isDeployed);
    }
}