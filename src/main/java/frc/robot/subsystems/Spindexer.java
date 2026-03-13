package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Spindexer extends SubsystemBase {

    // ===== HARDWARE =====
    // spindexerMotor is a TalonFXS (note the S) — needs a different config class
    private final TalonFXS spindexerMotor = new TalonFXS(54, Constants.kCANBus.getName());
    private final TalonFX  yeeterMotor    = new TalonFX(55, Constants.kCANBus.getName());

    // ===== CONTROL REQUESTS =====
    // VelocityVoltage = cruise control. Command a target RPS and the
    // motor holds it automatically regardless of battery level.
    // Positive velocity = forward.  Negative velocity = reverse.
    private final VelocityVoltage spindexerRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage yeeterRequest    = new VelocityVoltage(0.0);

    // ===== DEFAULT GAINS =====
    private static final double SPINDEXER_DEFAULT_TARGET_RPS = 58.0;
    private static final double SPINDEXER_DEFAULT_kV         = 0.10;
    private static final double SPINDEXER_DEFAULT_kS         = 0.0;
    private static final double SPINDEXER_DEFAULT_kP         = 0.0;

    private static final double YEETER_DEFAULT_TARGET_RPS    = 112.0;
    private static final double YEETER_DEFAULT_kV            = 0.10;
    private static final double YEETER_DEFAULT_kS            = 0.0;
    private static final double YEETER_DEFAULT_kP            = 0.0;

    // ===== NETWORKTABLES =====
    private final NetworkTable table = NetworkTableInstance.getDefault()
        .getTable("Spindexer");

    // ---- Tuning inputs (written from dashboard) ----
    private final DoubleSubscriber spindexerTargetSub = table
        .getDoubleTopic("Spindexer/Tuning/TargetRPS").subscribe(SPINDEXER_DEFAULT_TARGET_RPS);
    private final DoubleSubscriber spindexerKVSub = table
        .getDoubleTopic("Spindexer/Tuning/kV").subscribe(SPINDEXER_DEFAULT_kV);
    private final DoubleSubscriber spindexerKSSub = table
        .getDoubleTopic("Spindexer/Tuning/kS").subscribe(SPINDEXER_DEFAULT_kS);
    private final DoubleSubscriber spindexerKPSub = table
        .getDoubleTopic("Spindexer/Tuning/kP").subscribe(SPINDEXER_DEFAULT_kP);

    private final DoubleSubscriber yeeterTargetSub = table
        .getDoubleTopic("Yeeter/Tuning/TargetRPS").subscribe(YEETER_DEFAULT_TARGET_RPS);
    private final DoubleSubscriber yeeterKVSub = table
        .getDoubleTopic("Yeeter/Tuning/kV").subscribe(YEETER_DEFAULT_kV);
    private final DoubleSubscriber yeeterKSSub = table
        .getDoubleTopic("Yeeter/Tuning/kS").subscribe(YEETER_DEFAULT_kS);
    private final DoubleSubscriber yeeterKPSub = table
        .getDoubleTopic("Yeeter/Tuning/kP").subscribe(YEETER_DEFAULT_kP);

    // ---- Telemetry outputs (read-only in dashboard) ----
    private final DoublePublisher spindexerActualRPSPub = table
        .getDoubleTopic("Spindexer/ActualRPS").publish();
    private final DoublePublisher spindexerErrorPub = table
        .getDoubleTopic("Spindexer/RPS_Error").publish();
    private final DoublePublisher spindexerVoltsPub = table
        .getDoubleTopic("Spindexer/MotorVolts").publish();
    private final DoublePublisher yeeterActualRPSPub = table
        .getDoubleTopic("Yeeter/ActualRPS").publish();
    private final DoublePublisher yeeterErrorPub = table
        .getDoubleTopic("Yeeter/RPS_Error").publish();
    private final DoublePublisher yeeterVoltsPub = table
        .getDoubleTopic("Yeeter/MotorVolts").publish();

    // ===== GAIN TRACKING =====
    private double lastSpindexerKV = -1, lastSpindexerKS = -1, lastSpindexerKP = -1;
    private double lastYeeterKV    = -1, lastYeeterKS    = -1, lastYeeterKP    = -1;

    // ===== STATE FLAGS =====
    // These track what the motors are currently supposed to be doing so
    // periodic() can keep commanding the right speed without fighting
    // other commands.
    //
    // Think of it like a traffic light: only one state can be "active"
    // for each motor at a time.
    //   FORWARD = running forward (intake/shoot)
    //   REVERSE = running backward (clearing jams / purging)
    //   neither flag = stopped
    private boolean spindexerRunning = false;
    private boolean spindexerReverse = false;

    private boolean yeeterRunning = false;
    private boolean yeeterReverse = false;

    // ===== CONSTRUCTOR =====
    public Spindexer() {
        setName("Spindexer");

        // Configure spindexer (TalonFXS requires TalonFXSConfiguration)
        var spindexerConfig = new TalonFXSConfiguration();
        spindexerConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        spindexerConfig.Slot0.kV = SPINDEXER_DEFAULT_kV;
        spindexerConfig.Slot0.kS = SPINDEXER_DEFAULT_kS;
        spindexerConfig.Slot0.kP = SPINDEXER_DEFAULT_kP;
        spindexerMotor.getConfigurator().apply(spindexerConfig);

        // Configure yeeter (regular TalonFX uses TalonFXConfiguration)
        var yeeterConfig = new TalonFXConfiguration();
        yeeterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        yeeterConfig.Slot0.kV = YEETER_DEFAULT_kV;
        yeeterConfig.Slot0.kS = YEETER_DEFAULT_kS;
        yeeterConfig.Slot0.kP = YEETER_DEFAULT_kP;
        yeeterMotor.getConfigurator().apply(yeeterConfig);

        // =====================================================================
        // BUG FIX: Anonymous Publisher Garbage Collection
        // =====================================================================
        // OLD (broken) code looked like this:
        //   table.getDoubleTopic("Spindexer/Tuning/TargetRPS").publish().set(58.0);
        //
        // The problem: .publish() creates a Publisher object, but it was never
        // saved to a variable. Java's garbage collector can delete it almost
        // immediately, before the value ever reaches NetworkTables. The result
        // is that Elastic widgets show blank or stale default values on first open.
        //
        // Think of it like writing a letter, dropping it in the mailbox, but the
        // mailbox is deleted before the mail carrier comes — the letter never
        // arrives.
        //
        // THE FIX: Use getEntry().setDefaultDouble() instead.
        //   - getEntry() returns a persistent reference that won't be garbage collected.
        //   - setDefaultDouble() only writes the value if the key doesn't already
        //     exist, so it won't overwrite a value an operator set before restart.
        // =====================================================================
        table.getEntry("Spindexer/Tuning/TargetRPS").setDefaultDouble(SPINDEXER_DEFAULT_TARGET_RPS);
        table.getEntry("Spindexer/Tuning/kV").setDefaultDouble(SPINDEXER_DEFAULT_kV);
        table.getEntry("Spindexer/Tuning/kS").setDefaultDouble(SPINDEXER_DEFAULT_kS);
        table.getEntry("Spindexer/Tuning/kP").setDefaultDouble(SPINDEXER_DEFAULT_kP);
        table.getEntry("Yeeter/Tuning/TargetRPS").setDefaultDouble(YEETER_DEFAULT_TARGET_RPS);
        table.getEntry("Yeeter/Tuning/kV").setDefaultDouble(YEETER_DEFAULT_kV);
        table.getEntry("Yeeter/Tuning/kS").setDefaultDouble(YEETER_DEFAULT_kS);
        table.getEntry("Yeeter/Tuning/kP").setDefaultDouble(YEETER_DEFAULT_kP);
    }

    // ===== FORWARD METHODS (normal operation) =====

    /** Start spinning the spindexer carousel forward (intake / shoot direction). */
    public void runSpindexer() {
        spindexerRunning = true;
        spindexerReverse = false; // cancel any reverse that might have been active
        spindexerMotor.setControl(
            spindexerRequest.withVelocity(spindexerTargetSub.get()));
    }

    /** Stop the spindexer carousel. */
    public void stopSpindexer() {
        spindexerRunning = false;
        spindexerReverse = false;
        spindexerMotor.setControl(spindexerRequest.withVelocity(0.0));
    }

    /** Fire the yeeter to eject a ball (forward direction). */
    public void runYeeter() {
        yeeterRunning = true;
        yeeterReverse = false; // cancel any reverse
        yeeterMotor.setControl(
            yeeterRequest.withVelocity(yeeterTargetSub.get()));
    }

    /** Stop the yeeter. */
    public void stopYeeter() {
        yeeterRunning = false;
        yeeterReverse = false;
        yeeterMotor.setControl(yeeterRequest.withVelocity(0.0));
    }

    /** Stop both motors completely. */
    public void stopAll() {
        stopSpindexer();
        stopYeeter();
    }

    // ===== REVERSE METHODS =====
    // These run the motors BACKWARDS, which is useful for:
    //   - Clearing a jam (ball stuck in the mechanism)
    //   - Reversing an accidental intake
    //   - Unjamming the yeeter chute
    //
    // How they work: we negate the target RPS.
    // Example: normal forward = +58 RPS, reverse = -58 RPS.
    // The motor controller interprets negative RPS as spinning the other way.

    /**
     * Run the spindexer BACKWARDS (for jam clearing).
     * Call this inside a whileTrue() binding.
     */
    public void runSpindexerReverse() {
        spindexerRunning = false;
        spindexerReverse = true;
        spindexerMotor.setControl(
            spindexerRequest.withVelocity(-spindexerTargetSub.get()));
    }

    /**
     * Run the yeeter BACKWARDS (for jam clearing).
     * Call this inside a whileTrue() binding.
     */
    public void runYeeterReverse() {
        yeeterRunning = false;
        yeeterReverse = true;
        yeeterMotor.setControl(
            yeeterRequest.withVelocity(-yeeterTargetSub.get()));
    }

    // ===== PERIODIC =====
    @Override
    public void periodic() {

        // ---- Check if spindexer gains changed, re-apply if so ----
        double sKV = spindexerKVSub.get();
        double sKS = spindexerKSSub.get();
        double sKP = spindexerKPSub.get();
        if (sKV != lastSpindexerKV || sKS != lastSpindexerKS || sKP != lastSpindexerKP) {
            Slot0Configs gains = new Slot0Configs();
            gains.kV = sKV; gains.kS = sKS; gains.kP = sKP;
            spindexerMotor.getConfigurator().apply(gains);
            lastSpindexerKV = sKV; lastSpindexerKS = sKS; lastSpindexerKP = sKP;
        }

        // ---- Check if yeeter gains changed, re-apply if so ----
        double yKV = yeeterKVSub.get();
        double yKS = yeeterKSSub.get();
        double yKP = yeeterKPSub.get();
        if (yKV != lastYeeterKV || yKS != lastYeeterKS || yKP != lastYeeterKP) {
            Slot0Configs gains = new Slot0Configs();
            gains.kV = yKV; gains.kS = yKS; gains.kP = yKP;
            yeeterMotor.getConfigurator().apply(gains);
            lastYeeterKV = yKV; lastYeeterKS = yKS; lastYeeterKP = yKP;
        }

        // ---- Keep commanding the active speed/direction ----
        // Only one of the three states (forward, reverse, stopped) will
        // be active at a time for each motor.
        if (spindexerRunning) {
            spindexerMotor.setControl(
                spindexerRequest.withVelocity(spindexerTargetSub.get()));
        } else if (spindexerReverse) {
            spindexerMotor.setControl(
                spindexerRequest.withVelocity(-spindexerTargetSub.get()));
        }

        if (yeeterRunning) {
            yeeterMotor.setControl(
                yeeterRequest.withVelocity(yeeterTargetSub.get()));
        } else if (yeeterReverse) {
            yeeterMotor.setControl(
                yeeterRequest.withVelocity(-yeeterTargetSub.get()));
        }

        // ---- Publish telemetry ----
        double spindexerActual = spindexerMotor.getVelocity().getValueAsDouble();
        double yeeterActual    = yeeterMotor.getVelocity().getValueAsDouble();
        spindexerActualRPSPub.set(spindexerActual);
        spindexerErrorPub.set(spindexerTargetSub.get() - spindexerActual);
        spindexerVoltsPub.set(spindexerMotor.getMotorVoltage().getValueAsDouble());
        yeeterActualRPSPub.set(yeeterActual);
        yeeterErrorPub.set(yeeterTargetSub.get() - yeeterActual);
        yeeterVoltsPub.set(yeeterMotor.getMotorVoltage().getValueAsDouble());
    }
}