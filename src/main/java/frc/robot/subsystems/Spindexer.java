package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;

// CTRE Phoenix 6 imports for motor controllers
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class Spindexer extends SubsystemBase {

    // ===== HARDWARE =====
    // The number in parentheses is the CAN ID — the "address" of each motor.
    private final TalonFXS spindexerMotor = new TalonFXS(54, Constants.kCANBus.getName());
    private final TalonFX  yeeterMotor    = new TalonFX(55,  Constants.kCANBus.getName());

    // ===== CONTROL REQUESTS =====
    // "Order forms" that tell each motor what to do.
    // 0.0 means "start with zero power."
    private final DutyCycleOut spindexerRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut yeeterRequest    = new DutyCycleOut(0.0);

    // ===== CONSTANTS =====
    // Motor speeds: -1.0 (full reverse) to 1.0 (full forward)
    private static final double SPINDEXER_SPEED = 0.5;   // 50% power
    private static final double YEETER_SPEED    = 1.0;   // 100% power

    // ===== NETWORKTABLES PUBLISHERS =====
    // These broadcast motor values so AdvantageScope can see them.
    private final DoublePublisher spindexerOutputPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Spindexer/SpindexerOutput").publish();
    private final DoublePublisher yeeterOutputPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Spindexer/YeeterOutput").publish();


    // ===== CONSTRUCTOR =====
    public Spindexer() {
        setName("Spindexer");

        // Configure the yeeter motor to spin clockwise when given a positive value.
        // This mirrors the same pattern used in Shooter.java for the right motor.
        var yeeterConfig = new TalonFXConfiguration();
        yeeterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        yeeterMotor.getConfigurator().apply(yeeterConfig);
    }

    // ===== METHODS =====

    /** Start spinning the spindexer carousel */
    public void runSpindexer() {
        spindexerMotor.setControl(
            spindexerRequest.withOutput(SPINDEXER_SPEED));
    }

    /** Stop the spindexer carousel */
    public void stopSpindexer() {
        spindexerMotor.setControl(
            spindexerRequest.withOutput(0.0));
    }

    /** Fire the yeeter to eject a ball */
    public void runYeeter() {
        yeeterMotor.setControl(
            yeeterRequest.withOutput(YEETER_SPEED));
    }

    /** Stop the yeeter */
    public void stopYeeter() {
        yeeterMotor.setControl(
            yeeterRequest.withOutput(0.0));
    }

    /** Stop everything */
    public void stopAll() {
        stopSpindexer();
        stopYeeter();
    }


    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // Publish motor outputs to AdvantageScope
        spindexerOutputPub.set(
            spindexerMotor.getDutyCycle().getValueAsDouble());
        yeeterOutputPub.set(
            yeeterMotor.getDutyCycle().getValueAsDouble());
    }
}