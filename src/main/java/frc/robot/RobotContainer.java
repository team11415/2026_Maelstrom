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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private double MaxSpeed = 1.0 * Constants.kSpeedAt12Volts.in(MetersPerSecond);
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

    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();

    private final Vision vision = new Vision(
       () -> drivetrain.getState().Pose,
       (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs),
       () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);  

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
            Commands.sequence(
                // Step 1: Spin up the flywheel first
                Commands.runOnce(() -> {
                    shooter.runShooter();
                    leds.setShooting(true);
                }, shooter),

                // Step 2: Wait for flywheel to reach speed before feeding the ball
                new WaitCommand(0.25),

                // Step 3: Feed the ball in with spindexer and yeeter
                runEnd(
                    () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                    () -> { spindexer.stopAll(); shooter.stopShooter(); leds.setShooting(false); },
                    spindexer, shooter
                )
            ).withTimeout(4.0) // The whole sequence times out after 4 seconds total
        );

        NamedCommands.registerCommand("runWait2", new WaitCommand(2.0));
        NamedCommands.registerCommand("runWait4", new WaitCommand(4.0));

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

        // ===== RIGHT TRIGGER: AIM-ASSIST + SHOOT =====
        // Pressing the trigger does THREE things at once:
        //   1. Robot auto-rotates to face the best aim target
        //   2. Flywheel spins up to the correct speed for that distance
        //   3. After spin-up, spindexer and yeeter feed the ball
        // The driver can still drive freely with the left stick while all this happens.
        driver.rightTrigger(0.5)
            .whileTrue(
                Commands.parallel(

                    // ---- ARM 1: Aim-assist drive ----
                    // Overrides the default drive command's rotation only.
                    // The driver still controls X/Y translation with the left stick.
                    drivetrain.applyRequest(() -> {
                        Translation2d aimTarget = getAimTarget();
                        Translation2d robotPosition = drivetrain.getState().Pose
                            .getTranslation();

                        // Calculate the angle from the robot to the target
                        Translation2d robotToTarget = aimTarget.minus(robotPosition);
                        double angleToTargetRad = Math.atan2(
                            robotToTarget.getY(), robotToTarget.getX());

                        // Offset by 90° because the shooter faces LEFT
                        double desiredHeadingRad = angleToTargetRad
                            - Math.toRadians(Constants.SHOOTER_ANGLE_OFFSET_DEG);

                        return aimDrive
                            .withVelocityX(-driver.getLeftY() * MaxSpeed)
                            .withVelocityY(-driver.getLeftX() * MaxSpeed)
                            .withTargetDirection(new Rotation2d(desiredHeadingRad));
                    }),

                    // ---- ARM 2: Shoot sequence ----
                    // Runs at the same time as the aim-assist above.
                    Commands.sequence(
                        // Step 1: Spin up flywheel to distance-appropriate speed
                        Commands.runOnce(() -> {
                            Translation2d aimTarget = getAimTarget();
                            Translation2d robotPosition = drivetrain.getState().Pose
                                .getTranslation();
                            double distance = robotPosition.getDistance(aimTarget);
                            double autoSpeed = Constants.SHOOTER_SPEED_MAP.get(distance);
                            shooter.runShooterAtSpeed(autoSpeed);
                            leds.setShooting(true);
                        }, shooter),

                        // Step 2: Wait for flywheel to reach speed
                        new WaitCommand(0.25),

                        // Step 3: Feed the ball — runs until trigger is released
                        runEnd(
                            () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                            () -> { shooter.stopShooter(); spindexer.stopAll();
                                    leds.setShooting(false); },
                            shooter, spindexer
                        )
                    )
                )
            );
        

        // Left bumper: reset field-centric heading
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SysId routines (for characterization — hold back/start + Y/X)
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        
        // ===== TELEMETRY =====
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Returns the best aim target for the robot's current position and alliance.
     *
     * Decision logic:
     *
     *   BLUE alliance:
     *     x < 3.978m  →  shoot directly into BLUE_HUB
     *     x >= 3.978m AND y > 4.035m  →  pass to BLUE_PASS_LEFT
     *     x >= 3.978m AND y <= 4.035m →  pass to BLUE_PASS_RIGHT
     *
     *   RED alliance:
     *     x > 12.563m →  shoot directly into RED_HUB
     *     x <= 12.563m AND y < 4.035m  →  pass to RED_PASS_LEFT
     *     x <= 12.563m AND y >= 4.035m →  pass to RED_PASS_RIGHT
     */
    private Translation2d getAimTarget() {
        // Get the robot's current position on the field
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        double x = robotPos.getX();
        double y = robotPos.getY();

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() &&
                        alliance.get() == DriverStation.Alliance.Red;

        if (isRed) {
            // Red alliance logic
            if (x > Constants.RED_HUB_MIN_X) {
                return Constants.RED_HUB;
            } else if (y < Constants.PASS_SPLIT_Y) {
                return Constants.RED_PASS_LEFT;
            } else {
                return Constants.RED_PASS_RIGHT;
            }
        } else {
            // Blue alliance logic (also the default if alliance is unknown)
            if (x < Constants.BLUE_HUB_MAX_X) {
                return Constants.BLUE_HUB;
            } else if (y > Constants.PASS_SPLIT_Y) {
                return Constants.BLUE_PASS_LEFT;
            } else {
                return Constants.BLUE_PASS_RIGHT;
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Allows Robot.java to access the Vision subsystem
    // so it can control throttle on enable/disable.
    public Vision getVision() {
        return vision;
    }
}