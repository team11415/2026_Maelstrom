package frc.robot.subsystems;
import frc.robot.Constants;
// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;


public class Shooter extends SubsystemBase {

    // ===== HARDWARE =====
    // Two Kraken x60 motors, shafts pointed toward each other.
    // We call them "left" and "right" from the robot's perspective.
    // Both are on the CANivore bus — see Constants.kCANBus
    //private final TalonFX leftMotor  = new TalonFX(60, Constants.kCANBus.getName()); //not implemented yet - uncomment this and 5 other ines
    private final TalonFX rightMotor = new TalonFX(61, Constants.kCANBus.getName());

    // ===== CONTROL REQUESTS =====
    // We only need one request object — we'll send it to both motors.
    // The right motor is inverted in the constructor, so sending the
    // SAME positive value to both motors makes them spin the ball
    // in the same direction.
    private final DutyCycleOut shootRequest = new DutyCycleOut(0.0);

    // ===== CONSTANTS =====
    // Shooter speed. We'll start conservative and tune later.
    private static final double SHOOTER_SPEED = 0.70;  // 70% power

    // ===== NETWORKTABLES PUBLISHERS =====
    // Broadcast motor values to AdvantageScope
    private final DoublePublisher leftOutputPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Shooter/LeftOutput").publish();
    private final DoublePublisher rightOutputPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Shooter/RightOutput").publish();


    // ===== CONSTRUCTOR =====
    public Shooter() {
        setName("Shooter");

        // Configure the right motor to spin in the opposite direction.
        // Since the motors face each other, we need to invert one
        // so they both push the ball the same way.
        // Think of it like flipping a "reverse" switch on one motor.
        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(rightConfig);
    }


    // ===== METHODS =====

    /** Rev up the shooter wheels */
    public void runShooter() {
        //leftMotor.setControl(shootRequest.withOutput(SHOOTER_SPEED));
        rightMotor.setControl(shootRequest.withOutput(SHOOTER_SPEED));
    }

    /**
     * Run the shooter at a specific speed (for distance-based shooting).
     * @param speed Motor output from 0.0 to 1.0
     */
    public void runShooterAtSpeed(double speed) {
        // Clamp between 0 and 1 for safety
        speed = Math.max(0.0, Math.min(1.0, speed));
        //leftMotor.setControl(shootRequest.withOutput(speed));
        rightMotor.setControl(shootRequest.withOutput(speed));
    }

    /** Stop the shooter wheels */
    public void stopShooter() {
        //leftMotor.setControl(shootRequest.withOutput(0.0));
        rightMotor.setControl(shootRequest.withOutput(0.0));
    }


    // ===== PERIODIC =====
    @Override
    public void periodic() {
        // Publish motor outputs to AdvantageScope
        //leftOutputPub.set(
            //leftMotor.getDutyCycle().getValueAsDouble());
        rightOutputPub.set(
            rightMotor.getDutyCycle().getValueAsDouble());
    }
}