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
    // spindexerMotor is a TalonFXS (note the S) — different config type than TalonFX
    private final TalonFXS spindexerMotor = new TalonFXS(54, Constants.kCANBus.getName());
    private final TalonFX  yeeterMotor    = new TalonFX(55,  Constants.kCANBus.getName());

    // ===== CONTROL REQUESTS =====
    // VelocityVoltage = cruise control. Command a target RPS and the
    // motor holds it automatically regardless of battery level.
    private final VelocityVoltage spindexerRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage yeeterRequest    = new VelocityVoltage(0.0);

    // ===== DEFAULT GAINS =====
    // kV calculated from your measurements:
    //   Spindexer: 5.8V ÷ 58 RPS  = 0.10
    //   Yeeter:    11V  ÷ 112 RPS  = 0.098 → rounded to 0.10
    // These are great starting points — tune live from the dashboard.
    private static final double SPINDEXER_DEFAULT_TARGET_RPS = 58.0;
    private static final double SPINDEXER_DEFAULT_kV         = 0.10;
    private static final double SPINDEXER_DEFAULT_kS         = 0.0;
    private static final double SPINDEXER_DEFAULT_kP         = 0.0;

    private static final double YEETER_DEFAULT_TARGET_RPS    = 112.0;
    private static final double YEETER_DEFAULT_kV            = 0.10;
    private static final double YEETER_DEFAULT_kS            = 0.0;
    private static final double YEETER_DEFAULT_kP            = 0.0;

    // ===== NETWORKTABLES =====
    // All spindexer/yeeter entries live under "Spindexer/"
    private final NetworkTable table = NetworkTableInstance.getDefault()
        .getTable("Spindexer");

    // ---- Tuning inputs (you write these from the dashboard) ----
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
    // Remember last applied gains so we only re-apply when something changes.
    // Applying configs is an expensive CAN operation — don't spam it.
    private double lastSpindexerKV = -1, lastSpindexerKS = -1, lastSpindexerKP = -1;
    private double lastYeeterKV    = -1, lastYeeterKS    = -1, lastYeeterKP    = -1;

    // ===== RUNNING FLAGS =====
    // Track whether each motor is supposed to be spinning so periodic()
    // can keep commanding the latest target RPS if it changes live.
    private boolean spindexerRunning = false;
    private boolean yeeterRunning    = false;

    // ===== CONSTRUCTOR =====
    public Spindexer() {
        setName("Spindexer");

        // Configure spindexer — must use TalonFXSConfiguration (not TalonFXConfiguration)
        // because this motor is a TalonFXS, which has a different config class.
        var spindexerConfig = new TalonFXSConfiguration();
        spindexerConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        spindexerConfig.Slot0.kV = SPINDEXER_DEFAULT_kV;
        spindexerConfig.Slot0.kS = SPINDEXER_DEFAULT_kS;
        spindexerConfig.Slot0.kP = SPINDEXER_DEFAULT_kP;
        spindexerMotor.getConfigurator().apply(spindexerConfig);

        // Configure yeeter — regular TalonFX uses TalonFXConfiguration
        var yeeterConfig = new TalonFXConfiguration();
        yeeterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        yeeterConfig.Slot0.kV = YEETER_DEFAULT_kV;
        yeeterConfig.Slot0.kS = YEETER_DEFAULT_kS;
        yeeterConfig.Slot0.kP = YEETER_DEFAULT_kP;
        yeeterMotor.getConfigurator().apply(yeeterConfig);

        // Publish defaults to dashboard so entries appear immediately
        // when you open Elastic, rather than being blank
        table.getDoubleTopic("Spindexer/Tuning/TargetRPS").publish().set(SPINDEXER_DEFAULT_TARGET_RPS);
        table.getDoubleTopic("Spindexer/Tuning/kV").publish().set(SPINDEXER_DEFAULT_kV);
        table.getDoubleTopic("Spindexer/Tuning/kS").publish().set(SPINDEXER_DEFAULT_kS);
        table.getDoubleTopic("Spindexer/Tuning/kP").publish().set(SPINDEXER_DEFAULT_kP);

        table.getDoubleTopic("Yeeter/Tuning/TargetRPS").publish().set(YEETER_DEFAULT_TARGET_RPS);
        table.getDoubleTopic("Yeeter/Tuning/kV").publish().set(YEETER_DEFAULT_kV);
        table.getDoubleTopic("Yeeter/Tuning/kS").publish().set(YEETER_DEFAULT_kS);
        table.getDoubleTopic("Yeeter/Tuning/kP").publish().set(YEETER_DEFAULT_kP);
    }

    // ===== METHODS =====

    /** Start spinning the spindexer carousel */
    public void runSpindexer() {
        spindexerRunning = true;
        spindexerMotor.setControl(
            spindexerRequest.withVelocity(spindexerTargetSub.get()));
    }

    /** Stop the spindexer carousel */
    public void stopSpindexer() {
        spindexerRunning = false;
        spindexerMotor.setControl(spindexerRequest.withVelocity(0.0));
    }

    /** Fire the yeeter to eject a ball */
    public void runYeeter() {
        yeeterRunning = true;
        yeeterMotor.setControl(
            yeeterRequest.withVelocity(yeeterTargetSub.get()));
    }

    /** Stop the yeeter */
    public void stopYeeter() {
        yeeterRunning = false;
        yeeterMotor.setControl(yeeterRequest.withVelocity(0.0));
    }

    /** Stop everything */
    public void stopAll() {
        stopSpindexer();
        stopYeeter();
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
            gains.kV = sKV;
            gains.kS = sKS;
            gains.kP = sKP;
            spindexerMotor.getConfigurator().apply(gains);
            lastSpindexerKV = sKV;
            lastSpindexerKS = sKS;
            lastSpindexerKP = sKP;
        }

        // ---- Check if yeeter gains changed, re-apply if so ----
        double yKV = yeeterKVSub.get();
        double yKS = yeeterKSSub.get();
        double yKP = yeeterKPSub.get();
        if (yKV != lastYeeterKV || yKS != lastYeeterKS || yKP != lastYeeterKP) {
            Slot0Configs gains = new Slot0Configs();
            gains.kV = yKV;
            gains.kS = yKS;
            gains.kP = yKP;
            yeeterMotor.getConfigurator().apply(gains);
            lastYeeterKV = yKV;
            lastYeeterKS = yKS;
            lastYeeterKP = yKP;
        }

        // ---- If running, keep commanding the latest target RPS ----
        // This means if you adjust TargetRPS on the dashboard while
        // the motors are already spinning, it takes effect immediately.
        if (spindexerRunning) {
            spindexerMotor.setControl(
                spindexerRequest.withVelocity(spindexerTargetSub.get()));
        }
        if (yeeterRunning) {
            yeeterMotor.setControl(
                yeeterRequest.withVelocity(yeeterTargetSub.get()));
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