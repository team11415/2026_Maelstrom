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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
    // Change 1.0 to 0.2 for 20% speed during initial testing
    private double MaxSpeed = 0.2 * Constants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // This swerve request lets the driver control translation while
    // the robot automatically rotates to face a target angle.
    // Think of it as "cruise control for rotation."
    private final SwerveRequest.FieldCentricFacingAngle aimDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);    // Driver
    private final CommandXboxController operator = new CommandXboxController(1);  // Operator

    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();

    private final Vision vision = new Vision(
       () -> drivetrain.getState().Pose,
       (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs));

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
        NamedCommands.registerCommand("runShooter",
                    runEnd(
                        () -> { spindexer.runSpindexer(); spindexer.runYeeter(); shooter.runShooter(); leds.setShooting(true); },
                        () -> { spindexer.stopAll(); shooter.stopShooter(); leds.setShooting(false); },
                        spindexer, shooter
                    ).withTimeout(4.0)
                );

        NamedCommands.registerCommand("stopAll",
            Commands.runOnce(() -> { spindexer.stopAll(); shooter.stopShooter(); }, spindexer, shooter)
        );

        // Build the auto chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // ===== DEFAULT DRIVE COMMAND =====
        // Driver's left stick = translation, right stick = rotation
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Configure the heading PID for aim-assist mode.
        // This controls how aggressively the robot rotates to face the target.
        aimDrive.HeadingController.setPID(7.0, 0.0, 0.0);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // ===== DRIVER CONTROLS =====

        // A button: brake (X-lock wheels)
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B button: point wheels at direction of left stick
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // X button: AIM-ASSIST mode
        // Driver still controls translation (strafing) with the left stick,
        // but rotation is automatic — the robot turns so the shooter
        // (on the left side) points at the hub.
        driver.x().whileTrue(
            drivetrain.applyRequest(() -> {
                // Step 1: Figure out which hub we're aiming at
                Translation2d hubPosition = getTargetHub();

                // Step 2: Get our current position on the field
                Translation2d robotPosition = drivetrain.getState().Pose
                    .getTranslation();

                // Step 3: Calculate the angle FROM the robot TO the hub
                // atan2 gives us the field angle to the target
                Translation2d robotToHub = hubPosition.minus(robotPosition);
                double angleToHubRad = Math.atan2(
                    robotToHub.getY(), robotToHub.getX());

                // Step 4: Offset by 90° because the shooter faces LEFT
                // We need the robot's LEFT side to point at the hub,
                // so the robot heading must be 90° less than the target angle.
                double desiredHeadingRad = angleToHubRad
                    - Math.toRadians(Constants.SHOOTER_ANGLE_OFFSET_DEG);

                // Step 5: Build the drive request
                // Driver still controls translation (strafing),
                // but rotation locks onto the hub automatically
                return aimDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withTargetDirection(new Rotation2d(desiredHeadingRad));
            })
        );

        // Left bumper: reset field-centric heading
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SysId routines (for characterization — hold back/start + Y/X)
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ===== OPERATOR CONTROLS =====

        // Right trigger: shoot with distance-based flywheel speed
        // The spindexer, yeeter, and shooter all run together.
        // Flywheel speed is automatically adjusted based on distance to hub.
        operator.rightTrigger(0.5)
            .whileTrue(runEnd(
                () -> {
                    // Calculate distance to hub for auto-speed
                    Translation2d hubPosition = getTargetHub();
                    Translation2d robotPosition = drivetrain.getState().Pose
                        .getTranslation();
                    double distance = robotPosition.getDistance(hubPosition);

                    // Look up the right speed for this distance
                    double autoSpeed = Constants.SHOOTER_SPEED_MAP.get(distance);

                    // Run everything
                    shooter.runShooterAtSpeed(autoSpeed);
                    spindexer.runSpindexer();
                    spindexer.runYeeter();
                    leds.setShooting(true);
                },
                () -> {
                    shooter.stopShooter();
                    spindexer.stopAll();
                    leds.setShooting(false);
                },
                shooter, spindexer));

        // ===== TELEMETRY =====
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Returns the field position of OUR alliance's hub.
     * Blue alliance aims at the blue hub, red aims at the red hub.
     */
    private Translation2d getTargetHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Constants.RED_HUB;
        }
        return Constants.BLUE_HUB; // Default to blue
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}