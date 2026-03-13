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
 * ============================================================
 * HOW MOTIONMAGIC WORKS (simple explanation):
 * ============================================================
 *
 *  Normal position control just slams to the target and hopes for the best.
 *  MotionMagic is like cruise control: it smoothly ramps up to a cruise
 *  speed, holds it, then smoothly decelerates to stop at the target position.
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
 * ============================================================
 * USAGE (from RobotContainer.java):
 * ============================================================
 *
 *  // Left trigger held → extend and spin:
 *  driver.leftTrigger(0.5).whileTrue(
 *      runEnd(() -> intake.deployAndRun(), () -> intake.retractAndStop(), intake)
 *  );
 *
 *  // Back + Left trigger held → extend and spin roller BACKWARDS (eject):
 *  driver.back().and(driver.leftTrigger()).whileTrue(
 *      runEnd(() -> intake.deployAndRunReverse(), () -> intake.retractAndStop(), intake)
 *  );
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
    private final MotionMagicVoltage extendRequest =
        new MotionMagicVoltage(0.0).withSlot(0);

    // VelocityVoltage: command a target RPS for the roller.
    // Positive = intake direction.  Negative = reverse/eject direction.
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0.0);


    // =========================================================
    // POSITION CONSTANTS
    // =========================================================

    private static final double RETRACTED_ROTATIONS = 0.0;

    // >>> MEASURE THIS ON THE REAL ROBOT <<<
    private static final double EXTENDED_ROTATIONS = 8.0; // >>> MEASURE THIS <<<


    // =========================================================
    // MOTIONMAGIC TUNING DEFAULTS (extend/retract motors)
    // =========================================================

    private static final double DEFAULT_MM_CRUISE_VEL  = 2.0;
    private static final double DEFAULT_MM_ACCEL       = 4.0;
    private static final double DEFAULT_MM_JERK        = 0.0;

    private static final double DEFAULT_EXTEND_kV = 0.12;
    private static final double DEFAULT_EXTEND_kS = 0.0;
    private static final double DEFAULT_EXTEND_kP = 1.0;
    private static final double DEFAULT_EXTEND_kD = 0.0;


    // =========================================================
    // ROLLER TUNING DEFAULTS
    // =========================================================

    private static final double DEFAULT_ROLLER_TARGET_RPS = 50.0;
    private static final double DEFAULT_ROLLER_kV         = 0.12;
    private static final double DEFAULT_ROLLER_kS         = 0.0;
    private static final double DEFAULT_ROLLER_kP         = 0.1;


    // =========================================================
    // NETWORKTABLES SETUP
    // =========================================================

    private final NetworkTable table = NetworkTableInstance.getDefault()
        .getTable("Intake");

    // ---- Tuning inputs (writable from Elastic dashboard) ----
    private final DoubleSubscriber mmCruiseSub = table
        .getDoubleTopic("Tuning/MM_CruiseVelocity").subscribe(DEFAULT_MM_CRUISE_VEL);
    private final DoubleSubscriber mmAccelSub  = table
        .getDoubleTopic("Tuning/MM_Acceleration").subscribe(DEFAULT_MM_ACCEL);
    private final DoubleSubscriber mmJerkSub   = table
        .getDoubleTopic("Tuning/MM_Jerk").subscribe(DEFAULT_MM_JERK);

    private final DoubleSubscriber extKVSub = table
        .getDoubleTopic("Tuning/Extend_kV").subscribe(DEFAULT_EXTEND_kV);
    private final DoubleSubscriber extKSSub = table
        .getDoubleTopic("Tuning/Extend_kS").subscribe(DEFAULT_EXTEND_kS);
    private final DoubleSubscriber extKPSub = table
        .getDoubleTopic("Tuning/Extend_kP").subscribe(DEFAULT_EXTEND_kP);
    private final DoubleSubscriber extKDSub = table
        .getDoubleTopic("Tuning/Extend_kD").subscribe(DEFAULT_EXTEND_kD);

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

    private final BooleanPublisher leftFwdLimitPub  = table
        .getBooleanTopic("LeftExtend_AtExtended").publish();
    private final BooleanPublisher leftRevLimitPub  = table
        .getBooleanTopic("LeftExtend_AtRetracted").publish();
    private final BooleanPublisher rightFwdLimitPub = table
        .getBooleanTopic("RightExtend_AtExtended").publish();
    private final BooleanPublisher rightRevLimitPub = table
        .getBooleanTopic("RightExtend_AtRetracted").publish();

    private final BooleanPublisher deployedPub = table
        .getBooleanTopic("IsDeployed").publish();


    // =========================================================
    // GAIN TRACKING
    // =========================================================
    private double lastExtKV = -1, lastExtKS = -1, lastExtKP = -1, lastExtKD = -1;
    private double lastMmCruise = -1, lastMmAccel = -1, lastMmJerk = -1;
    private double lastRollerKV = -1, lastRollerKS = -1, lastRollerKP = -1;

    // =========================================================
    // STATE TRACKING
    // =========================================================
    private boolean isDeployed = false;

    // Roller state — only one of these should be true at a time:
    //   rollerRunning = spinning forward (normal intake)
    //   rollerReverse = spinning backward (eject / unjam)
    //   both false    = stopped
    private boolean rollerRunning = false;
    private boolean rollerReverse = false;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================
    public Intake() {
        setName("Intake");

        configureExtendMotor(leftExtend,  false);
        configureExtendMotor(rightExtend, true);

        configureRollerMotor();
        publishDefaults();

        // Start retracted on boot — safe starting position
        retract();
    }


    // =========================================================
    // MOTOR CONFIGURATION HELPERS
    // =========================================================

    /**
     * Configures one TalonFXS extend motor.
     *
     * Key things we set up:
     *   1. MotorArrangement = Minion_JST  (required for Minion motors)
     *   2. Inversion direction
     *   3. Brake mode so the intake holds position when stopped
     *   4. Hardware limit switches (forward = extended, reverse = retracted)
     *   5. Auto-zero encoder on reverse limit (retracted = always 0 rotations)
     *   6. Starting Slot0 gains and MotionMagic parameters
     */
    private void configureExtendMotor(TalonFXS motor, boolean inverted) {
        var config = new TalonFXSConfiguration();

        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var limitConfig = config.HardwareLimitSwitch;
        limitConfig.ForwardLimitEnable = true;
        limitConfig.ForwardLimitType   = ForwardLimitTypeValue.NormallyOpen;
        limitConfig.ReverseLimitEnable = true;
        limitConfig.ReverseLimitType   = ReverseLimitTypeValue.NormallyOpen;

        // Auto-zero when fully retracted — like a GPS recalibrating at your driveway
        limitConfig.ReverseLimitAutosetPositionEnable = true;
        limitConfig.ReverseLimitAutosetPositionValue  = 0.0;

        config.Slot0.kV = DEFAULT_EXTEND_kV;
        config.Slot0.kS = DEFAULT_EXTEND_kS;
        config.Slot0.kP = DEFAULT_EXTEND_kP;
        config.Slot0.kD = DEFAULT_EXTEND_kD;
        config.Slot0.kI = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MM_CRUISE_VEL;
        config.MotionMagic.MotionMagicAcceleration   = DEFAULT_MM_ACCEL;
        config.MotionMagic.MotionMagicJerk           = DEFAULT_MM_JERK;

        motor.getConfigurator().apply(config);
    }

    /** Configures the Kraken x44 roller motor (TalonFX, ID 52). */
    private void configureRollerMotor() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kV = DEFAULT_ROLLER_kV;
        config.Slot0.kS = DEFAULT_ROLLER_kS;
        config.Slot0.kP = DEFAULT_ROLLER_kP;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        roller.getConfigurator().apply(config);
    }

    /**
     * Publishes default tuning values to NetworkTables so Elastic widgets
     * show numbers instead of blank boxes on first open.
     *
     * =====================================================================
     * BUG FIX: Anonymous Publisher Garbage Collection
     * =====================================================================
     * OLD (broken) code looked like this:
     *   table.getDoubleTopic("Tuning/MM_CruiseVelocity").publish().set(2.0);
     *
     * The problem: .publish() creates a Publisher object, but it was never
     * saved to a variable. Java's garbage collector can delete it almost
     * immediately, before the value ever reaches NetworkTables.
     * Think of it like writing a letter and dropping it in a mailbox that
     * gets demolished before the mail carrier arrives — the letter never
     * gets delivered.
     *
     * THE FIX: Use getEntry().setDefaultDouble() instead.
     *   - getEntry() is backed by a persistent handle that won't be collected.
     *   - setDefaultDouble() only writes if the key doesn't already exist,
     *     so it won't overwrite a value the operator set before a restart.
     * =====================================================================
     */
    private void publishDefaults() {
        table.getEntry("Tuning/MM_CruiseVelocity").setDefaultDouble(DEFAULT_MM_CRUISE_VEL);
        table.getEntry("Tuning/MM_Acceleration").setDefaultDouble(DEFAULT_MM_ACCEL);
        table.getEntry("Tuning/MM_Jerk").setDefaultDouble(DEFAULT_MM_JERK);
        table.getEntry("Tuning/Extend_kV").setDefaultDouble(DEFAULT_EXTEND_kV);
        table.getEntry("Tuning/Extend_kS").setDefaultDouble(DEFAULT_EXTEND_kS);
        table.getEntry("Tuning/Extend_kP").setDefaultDouble(DEFAULT_EXTEND_kP);
        table.getEntry("Tuning/Extend_kD").setDefaultDouble(DEFAULT_EXTEND_kD);
        table.getEntry("Tuning/Roller_TargetRPS").setDefaultDouble(DEFAULT_ROLLER_TARGET_RPS);
        table.getEntry("Tuning/Roller_kV").setDefaultDouble(DEFAULT_ROLLER_kV);
        table.getEntry("Tuning/Roller_kS").setDefaultDouble(DEFAULT_ROLLER_kS);
        table.getEntry("Tuning/Roller_kP").setDefaultDouble(DEFAULT_ROLLER_kP);
    }


    // =========================================================
    // PUBLIC CONTROL METHODS — FORWARD (normal intake operation)
    // =========================================================

    /**
     * Commands the intake to extend to the deployed position.
     * Uses MotionMagic for smooth, controlled movement.
     */
    public void extend() {
        isDeployed = true;
        leftExtend.setControl(extendRequest.withPosition(EXTENDED_ROTATIONS));
        rightExtend.setControl(extendRequest.withPosition(EXTENDED_ROTATIONS));
    }

    /**
     * Commands the intake to retract fully inside the robot.
     * The hardware reverse limit switch stops the motors and zeros the encoder.
     */
    public void retract() {
        isDeployed = false;
        leftExtend.setControl(extendRequest.withPosition(RETRACTED_ROTATIONS));
        rightExtend.setControl(extendRequest.withPosition(RETRACTED_ROTATIONS));
    }

    /**
     * Spins the intake roller FORWARD at the dashboard-set target RPS.
     * Call alongside extend() to intake a game piece.
     */
    public void runRoller() {
        rollerRunning = true;
        rollerReverse = false; // clear reverse flag so periodic() doesn't fight
        roller.setControl(rollerRequest.withVelocity(rollerRPSSub.get()));
    }

    /** Stops the intake roller completely. */
    public void stopRoller() {
        rollerRunning = false;
        rollerReverse = false;
        roller.setControl(rollerRequest.withVelocity(0.0));
    }

    /**
     * Convenience method: extend the arm AND spin the roller forward at the same time.
     * Designed to be called inside a whileTrue() command binding.
     */
    public void deployAndRun() {
        extend();
        runRoller();
    }

    /**
     * Convenience method: retract the arm AND stop the roller at the same time.
     * Designed to be called as the "end" action of a whileTrue() binding.
     */
    public void retractAndStop() {
        retract();
        stopRoller();
    }


    // =========================================================
    // PUBLIC CONTROL METHODS — REVERSE (eject)
    // =========================================================

    /**
     * Spins the intake roller BACKWARDS at the dashboard-set target RPS.
     * This pushes a game piece back out of the intake.
     */
    public void runRollerReverse() {
        rollerRunning = false;
        rollerReverse = true;
        roller.setControl(rollerRequest.withVelocity(-rollerRPSSub.get()));
    }

    /**
     * Convenience method: extend the arm AND spin the roller BACKWARDS.
     * Used with Back + Left Trigger to eject a game piece.
     */
    public void deployAndRunReverse() {
        extend();
        runRollerReverse();
    }


    // =========================================================
    // STATUS QUERIES
    // =========================================================

    /**
     * Returns true if the intake is confirmed fully retracted (reverse limit triggered).
     */
    public boolean isFullyRetracted() {
        return leftExtend.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Returns true if the intake is confirmed fully extended (forward limit triggered).
     */
    public boolean isFullyExtended() {
        return leftExtend.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }


    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {

        // ---- STEP 1: Check if extend gains or MotionMagic params changed ----
        double kV = extKVSub.get(), kS = extKSSub.get(),
               kP = extKPSub.get(), kD = extKDSub.get();
        double cruise = mmCruiseSub.get(), accel = mmAccelSub.get(), jerk = mmJerkSub.get();

        boolean gainsChanged = (kV != lastExtKV || kS != lastExtKS ||
                                kP != lastExtKP || kD != lastExtKD);
        boolean mmChanged    = (cruise != lastMmCruise || accel != lastMmAccel ||
                                jerk != lastMmJerk);

        if (gainsChanged || mmChanged) {
            var slot0 = new Slot0Configs();
            slot0.kV = kV; slot0.kS = kS; slot0.kP = kP; slot0.kD = kD; slot0.kI = 0.0;

            var mm = new MotionMagicConfigs();
            mm.MotionMagicCruiseVelocity = cruise;
            mm.MotionMagicAcceleration   = accel;
            mm.MotionMagicJerk           = jerk;

            leftExtend.getConfigurator().apply(slot0);
            leftExtend.getConfigurator().apply(mm);
            rightExtend.getConfigurator().apply(slot0);
            rightExtend.getConfigurator().apply(mm);

            lastExtKV = kV; lastExtKS = kS; lastExtKP = kP; lastExtKD = kD;
            lastMmCruise = cruise; lastMmAccel = accel; lastMmJerk = jerk;
        }

        // ---- STEP 2: Check if roller gains changed ----
        double rKV = rollerKVSub.get(), rKS = rollerKSSub.get(), rKP = rollerKPSub.get();
        if (rKV != lastRollerKV || rKS != lastRollerKS || rKP != lastRollerKP) {
            var slot0 = new Slot0Configs();
            slot0.kV = rKV; slot0.kS = rKS; slot0.kP = rKP; slot0.kI = 0.0; slot0.kD = 0.0;
            roller.getConfigurator().apply(slot0);
            lastRollerKV = rKV; lastRollerKS = rKS; lastRollerKP = rKP;
        }

        // ---- STEP 3: Keep commanding the roller at its current direction ----
        if (rollerRunning) {
            roller.setControl(rollerRequest.withVelocity(rollerRPSSub.get()));
        } else if (rollerReverse) {
            roller.setControl(rollerRequest.withVelocity(-rollerRPSSub.get()));
        }

        // ---- STEP 4: Publish telemetry ----
        leftPosPub.set(leftExtend.getPosition().getValueAsDouble());
        rightPosPub.set(rightExtend.getPosition().getValueAsDouble());

        double actualRPS = roller.getVelocity().getValueAsDouble();
        rollerRPSPub.set(actualRPS);

        // =====================================================================
        // BUG FIX: Roller error showed a false reading when stopped
        // =====================================================================
        // OLD (broken) code:
        //   rollerErrorPub.set(rollerRPSSub.get() - actualRPS);
        //
        // The problem: when the roller is deliberately stopped, actualRPS is 0
        // but rollerRPSSub.get() returns the full target (e.g. 50 RPS). The
        // dashboard then shows "50 RPS of error" even though nothing is wrong —
        // the roller is intentionally off. This makes it hard to tell the
        // difference between "stopped on purpose" and "running but lagging."
        //
        // THE FIX: Only compute error relative to the actual target when the
        // roller is actively running. When stopped, report 0 error.
        // Think of it like a car's speedometer: it shouldn't say "you're 60 mph
        // below your destination speed" when you're parked in your driveway.
        // =====================================================================
        double targetForError = (rollerRunning || rollerReverse) ? rollerRPSSub.get() : 0.0;
        rollerErrorPub.set(targetForError - actualRPS);

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