// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

public class RobotContainer {
    private double MaxSpeed = 1.0 * Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed); // comment to disable telemetry logging (performance impact)

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();

    // private final Vision vision = new Vision(
    //    () -> drivetrain.getState().Pose, // The () -> arrow is the "supplier" — it's saying "whenever you need the pose, call this function."
    //    (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs));
    
    private final Spindexer spindexer = new Spindexer();
    private final Shooter shooter = new Shooter();
    private final LEDs leds = new LEDs();

    // The auto chooser lets you pick which autonomous routine to run
    // from the dashboard (or Sim GUI). PathPlanner automatically populates
    // it with all the autos you've created in the PathPlanner app.
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        
        // Register named commands so PathPlanner can trigger them during auto.
        // The string name here is what you'll pick in the PathPlanner app.
        NamedCommands.registerCommand("runShooter", 
                    runEnd(
                        () -> { spindexer.runSpindexer(); spindexer.runYeeter(); shooter.runShooter(); },
                        () -> { spindexer.stopAll(); shooter.stopShooter();},
                        spindexer, shooter
                    ).withTimeout(4.0)
                );

        NamedCommands.registerCommand("stopAll",
            Commands.runOnce(() -> { spindexer.stopAll(); shooter.stopShooter(); }, spindexer, shooter)
        );
        NamedCommands.registerCommand("ledsOff", 
            Commands.runOnce(() -> leds.turnOff(), leds));
        NamedCommands.registerCommand("ledsSolid", 
            Commands.runOnce(() -> leds.runSolid(), leds));
            
        // Build the auto chooser — this finds all the autos you've made
        // in the PathPlanner app and puts them in a dropdown menu
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )

        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Spin the spindexer while the X button is held down
        joystick.x()
            .whileTrue(runEnd(
                () -> { spindexer.runSpindexer(); spindexer.runYeeter(); shooter.runShooter(); },
                () -> { spindexer.stopAll(); shooter.stopShooter(); },
                spindexer, shooter));

        drivetrain.registerTelemetry(logger::telemeterize); // comment to disable telemetry logging (performance impact)

        // LEDs default to chase animation
        leds.setDefaultCommand(leds.run(() -> leds.runChase()));
    }

    public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }
