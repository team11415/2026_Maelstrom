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
 *  driver.leftTrigger(0.5).whileTrue(intake.extendAndRunCommand());
 *
 *  // Back + A held → extend and spin roller BACKWARDS (eject game piece):
 *  driver.back().and(driver.a()).whileTrue(
 *      runEnd(
 *          () -> intake.deployAndRunReverse(),
 *          () -> intake.retractAndStop(),
 *          intake
 *      )
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

    private static final double DEFAULT_ROLLER_TARGET_RPS = 50.0;   // setting this higher may result in unwanted oscillation
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
    private boolean rollerReverse = false;  // NEW: tracks reverse state


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
    // PUBLIC CONTROL METHODS — REVERSE (Request #5)
    // =========================================================
    // These run the roller BACKWARDS, which pushes a game piece back
    // OUT through the intake instead of pulling it in.
    //
    // WHY YOU'D NEED THIS:
    //   - You accidentally intaked the wrong game piece
    //   - A game piece is stuck partway in and needs to be ejected
    //   - You want to hand a piece to another robot through the intake
    //
    // HOW IT WORKS:
    //   We command a NEGATIVE RPS value to the roller motor.
    //   The motor controller treats negative velocity as spinning in
    //   the opposite direction. It's exactly the same as forward,
    //   just with the sign flipped — like putting a fan in reverse.
    //
    //   The intake arm EXTENDS when reversing so the game piece has
    //   somewhere to go as it gets pushed out. If you tried to eject
    //   while retracted, the piece would have nowhere to go and could jam.
    //
    // BUTTON COMBO: Back + A (both held simultaneously)
    //   Released → retractAndStop() is called automatically by runEnd()
    // =========================================================

    /**
     * Spins the intake roller BACKWARDS at the dashboard-set target RPS.
     *
     * This pushes a game piece back out of the intake.
     * Call alongside extend() — we extend the arm so the game piece
     * has a clear path to exit the robot.
     */
    public void runRollerReverse() {
        rollerRunning = false; // clear forward flag so periodic() doesn't fight
        rollerReverse = true;
        // Negate the target RPS — same speed, opposite direction.
        // Example: normal is +40 RPS intake, reverse is -40 RPS eject.
        roller.setControl(rollerRequest.withVelocity(-rollerRPSSub.get()));
    }

    /**
     * Convenience method: extend the arm AND spin the roller BACKWARDS.
     *
     * This is the "running" action for the Back + A reverse intake combo.
     * Designed to be used with runEnd():
     *
     *   runEnd(
     *       () -> intake.deployAndRunReverse(),   // while buttons held
     *       () -> intake.retractAndStop(),        // when buttons released
     *       intake
     *   )
     *
     * The intake arm extends so the ejected game piece has a clear path out.
     * When the buttons are released, retractAndStop() tucks the arm back in.
     */
    public void deployAndRunReverse() {
        extend();          // arm out so the piece has somewhere to go
        runRollerReverse(); // roller backwards to push the piece out
    }


    // =========================================================
    // STATUS QUERIES
    // =========================================================

    /**
     * Returns true if the intake is confirmed fully retracted (reverse limit triggered).
     * Can be used to interlock other mechanisms.
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
        // Only one of rollerRunning or rollerReverse will be true at a time.
        // If neither is true, the roller was stopped and we don't touch it.
        if (rollerRunning) {
            // Forward — re-command in case Roller_TargetRPS changed on dashboard
            roller.setControl(rollerRequest.withVelocity(rollerRPSSub.get()));
        } else if (rollerReverse) {
            // Reverse — re-command with the negative speed
            roller.setControl(rollerRequest.withVelocity(-rollerRPSSub.get()));
        }

        // ---- STEP 4: Publish telemetry ----
        leftPosPub.set(leftExtend.getPosition().getValueAsDouble());
        rightPosPub.set(rightExtend.getPosition().getValueAsDouble());

        double actualRPS = roller.getVelocity().getValueAsDouble();
        rollerRPSPub.set(actualRPS);
        rollerErrorPub.set(rollerRPSSub.get() - actualRPS);

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