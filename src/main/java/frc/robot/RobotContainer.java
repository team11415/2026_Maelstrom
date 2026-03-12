// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

public class RobotContainer {

    private final double MaxSpeed       = 1.0 * Constants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    point = new SwerveRequest.PointWheelsAt();

    // This swerve request lets the driver control translation while
    // the robot automatically rotates to face a target angle.
    private final SwerveRequest.FieldCentricFacingAngle aimDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();
    
    @SuppressWarnings("unused")
    private final DashboardTelemetry dashboardTelemetry = new DashboardTelemetry(drivetrain);


    private final Vision vision = new Vision(
        () -> drivetrain.getState().Pose,
        (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs),
        () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);

    private final Spindexer spindexer = new Spindexer();
    private final Shooter   shooter   = new Shooter();
    private final LEDs      leds      = new LEDs();
    private final Intake    intake    = new Intake();

    
    // =========================================================================
    // AIM ASSIST TOGGLE  (Request #3)
    // =========================================================================
    // This is the NetworkTables entry for the Aim Assist on/off toggle.
    //
    // IMPORTANT — This is DIFFERENT from the Vision toggles:
    //
    //   VisionEnabled (in Vision.java)
    //     → Controls whether the Limelight cameras update the drivetrain's
    //       position estimate (pose estimation). Turning this off means the
    //       robot relies only on wheel odometry + Pigeon gyro for knowing
    //       where it is on the field. The cameras still run, just ignored.
    //
    //   AimAssistEnabled (this one, in RobotContainer.java)
    //     → Controls whether pressing the RIGHT TRIGGER makes the robot
    //       auto-ROTATE to face the shooting target. If off, the right
    //       trigger still spins up the shooter and feeds the ball — the
    //       driver just has to aim manually with the right stick.
    //
    // You can have vision ON but aim assist OFF (robot knows where it is
    // but doesn't auto-rotate). You can also have vision OFF and aim assist
    // ON (auto-rotates using wheel odometry position — less accurate but
    // still functional).
    //
    // In Elastic: browse to /ControlFlags/AimAssistEnabled and add as a
    // Toggle Switch widget.
    // =========================================================================
    private final NetworkTableEntry aimAssistEnabledEntry;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // ---- Set up the Aim Assist toggle in NetworkTables ----
        // This lives in the same "ControlFlags" table as the Vision toggles
        // so all your driver toggles are in one place in Elastic.
        aimAssistEnabledEntry = NetworkTableInstance.getDefault()
            .getTable("ControlFlags").getEntry("AimAssistEnabled");
        aimAssistEnabledEntry.setDefaultBoolean(true); // starts enabled

        configureBindings();

        // Register named commands for PathPlanner auto
        NamedCommands.registerCommand("runShooter",
            Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.runShooter();
                    leds.setShooting(true);
                }, shooter),
                new WaitCommand(0.25),
                runEnd(
                    () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                    () -> { spindexer.stopAll(); shooter.stopShooter(); leds.setShooting(false); },
                    spindexer, shooter
                )
            ).withTimeout(4.0)
        );
        // ===== NAMED COMMAND: RUN INTAKE (for PathPlanner auto) =====
        //
        //
        // runEnd() works like a light switch with two settings:
        //   "ON"  action → called every loop while the command is running
        //   "OFF" action → called once when the command is interrupted or times out

        NamedCommands.registerCommand("runIntake",
            runEnd(
                () -> {
                    intake.deployAndRun();       // extend arm + spin roller forward
                    leds.setIntaking(true);      // tell LEDs: strobe orange please
                },
                () -> {
                    intake.retractAndStop();     // retract arm + stop roller
                    leds.setIntaking(false);     // tell LEDs: go back to normal
                },
                intake                           // intake is the only subsystem "claimed"
                                                // (leds manages itself via periodic flags)
            )
        );

        NamedCommands.registerCommand("runWait2",  new WaitCommand(2.0));
        NamedCommands.registerCommand("runWait4",  new WaitCommand(4.0));
        NamedCommands.registerCommand("stopAll",
            Commands.runOnce(() -> { spindexer.stopAll(); shooter.stopShooter(); }, spindexer, shooter)
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        // ===== DEFAULT DRIVE COMMAND =====
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // Idle while disabled — applies neutral mode to drive motors
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Configure the heading PID for aim-assist rotation
        aimDrive.HeadingController.setPID(7.0, 0.0, 0.0);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // ===== A BUTTON: X-LOCK WHEELS =====
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // ===== B BUTTON: POINT WHEELS AT LEFT STICK DIRECTION =====
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // ===== LEFT TRIGGER: RUN INTAKE =====
        // Holding the left trigger extends the intake arm and spins the roller
        // forward to pull in a game piece.
        //
        // When the trigger is released, the arm automatically retracts and the
        // roller stops — you don't need to press anything else.
        //
        // This uses runEnd(), which works like this:
        //   - While trigger is held  → deployAndRun() is called (extend + spin roller in)
        //   - When trigger releases  → retractAndStop() is called (retract + stop roller)
        driver.leftTrigger(0.5).whileTrue(
            runEnd(
                () -> intake.deployAndRun(),    // extend arm + spin roller forward
                () -> intake.retractAndStop(),  // retract arm + stop roller on release
                intake                          // declare intake as the required subsystem
            )
        );

        // ===== RIGHT TRIGGER: AIM-ASSIST + SHOOT (modified for Request #3) =====
        //
        // Pressing the trigger does:
        //   If AIM ASSIST IS ON:
        //     1. Robot auto-rotates to face the best target
        //     2. Flywheel spins up to correct speed for that distance
        //     3. Spindexer + yeeter feed the ball
        //
        //   If AIM ASSIST IS OFF:
        //     1. Normal driver-controlled drive (you steer manually)
        //     2. Flywheel still spins up and ball is fed automatically
        //     (You just have to point the robot yourself with the right stick)
        //
        // The key difference is only in ARM 1 below — ARM 2 (the shoot
        // sequence) always runs regardless of the aim assist setting.
        driver.rightTrigger(0.5)
            .whileTrue(
                Commands.parallel(

                    // ---- ARM 1: Drive control during trigger hold ----
                    // Checks aim assist flag INSIDE the lambda so it
                    // re-evaluates every 20ms — you can toggle mid-match.
                    drivetrain.applyRequest(() -> {

                        boolean aimAssistOn = aimAssistEnabledEntry.getBoolean(true);

                        if (!aimAssistOn) {
                            // Aim assist OFF: hand rotation back to the driver.
                            // Shooter still fires in ARM 2 below; driver aims manually.
                            return drive
                                .withVelocityX(-driver.getLeftY() * MaxSpeed)
                                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                .withRotationalRate(-driver.getRightX() * MaxAngularRate);
                        }

                        // Aim assist ON: calculate the angle to the target and auto-rotate.
                        Translation2d aimTarget     = getLeadTarget(getAimTarget());
                        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();

                        Translation2d robotToTarget = aimTarget.minus(robotPosition);
                        double angleToTargetRad = Math.atan2(
                            robotToTarget.getY(), robotToTarget.getX());

                        // Offset by 90° because the shooter faces LEFT side of the robot
                        double desiredHeadingRad = angleToTargetRad
                            - Math.toRadians(Constants.SHOOTER_ANGLE_OFFSET_DEG);

                        return aimDrive
                            .withVelocityX(-driver.getLeftY() * MaxSpeed)
                            .withVelocityY(-driver.getLeftX() * MaxSpeed)
                            .withTargetDirection(new Rotation2d(desiredHeadingRad));
                    }),

                    // ---- ARM 2: Shoot sequence (runs regardless of aim assist) ----
                    Commands.sequence(
                        // Step 1: Spin up flywheel to distance-appropriate speed
                        Commands.runOnce(() -> {
                            Translation2d realTarget    = getAimTarget();
                            Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
                            double distance  = robotPosition.getDistance(realTarget);
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

        // ===== LEFT BUMPER: RESET FIELD-CENTRIC HEADING =====
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // =========================================================================
        // START + LEFT BUMPER: MANUAL POSE RESET TO KNOWN FIELD POSITION (Request #4)
        // =========================================================================
        // HOW TO USE:
        //   1. Drive the robot to a pre-agreed spot on the field
        //      (e.g., a line, corner, or wall you can easily return to).
        //   2. Hold START, then press LEFT BUMPER.
        //   3. The robot's X,Y position snaps to the coordinates defined in
        //      Constants.BLUE_RESET_POSITION or Constants.RED_RESET_POSITION.
        //
        // The ROTATION is NOT changed — the Pigeon gyro still knows which
        // way the robot is facing. Only the X,Y translation is corrected.
        //
        // WHY THIS IS USEFUL:
        //   If wheel slip has caused odometry drift, or vision had no tags
        //   early in the match, this "re-anchors" the robot's known location
        //   to a spot you physically drove to. Like pressing "recalibrate GPS."
        //
        // NOTE: Edit Constants.BLUE_RESET_POSITION and Constants.RED_RESET_POSITION
        //       to match wherever you actually want to reset from!
        // =========================================================================
        driver.start().and(driver.leftBumper()).onTrue(
            drivetrain.runOnce(() -> {

                // Find out which alliance we're on (IMPORTANT — Red and Blue
                // sides of the field are mirrored, so coordinates differ!)
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() &&
                    alliance.get() == DriverStation.Alliance.Red;

                // Pick the right known position for our alliance
                Translation2d knownPosition = isRed
                    ? Constants.RED_RESET_POSITION    // Red alliance reset spot
                    : Constants.BLUE_RESET_POSITION;  // Blue alliance reset spot

                // Keep the current gyro heading (we trust the gyro for rotation),
                // but force X,Y to the known physical coordinates.
                Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
                Pose2d resetPose = new Pose2d(knownPosition, currentHeading);

                // Tell the drivetrain "you are HERE right now"
                drivetrain.resetPose(resetPose);
            })
        );

        // =========================================================================
        // BACK + REVERSE SPINDEXER AND YEETER (Request #6)
        // =========================================================================
        // Hold BACK + RIGHT TRIGGER to run both the spindexer carousel and the yeeter wheel
        // in REVERSE.
        //
        // Both motors run in reverse as long as both buttons are held.
        // As soon as you release either button, both motors stop.
        // =========================================================================
        driver.back().and(driver.rightTrigger()).whileTrue(
            runEnd(
                () -> {
                    // Run spindexer carousel backwards
                    spindexer.runSpindexerReverse();
                    // Run yeeter wheel backwards
                    spindexer.runYeeterReverse();
                },
                () -> {
                    // STOP both motors when the buttons are released
                    spindexer.stopAll();
                },
                spindexer // declare spindexer as the required subsystem
            )
        );

        // =========================================================================
        // BACK + LEFT TRIGGER: REVERSE INTAKE (Request #5)
        // =========================================================================
        // Hold BACK + LEFT TRIGGER to extend the intake arm and spin the roller BACKWARDS.
        // This pushes a game piece back out through the intake opening.
        //
        // WHY WE EXTEND WHILE REVERSING:
        //   The arm extends outward so the ejected piece has a clear path to
        //   exit the robot. If you tried to eject while retracted, the piece
        //   would be trapped inside with nowhere to go.
        //
        // When BOTH buttons are released, the arm automatically retracts and
        // the roller stops — this happens because runEnd() calls
        // retractAndStop() as its "end" action.
        // =========================================================================
        driver.back().and(driver.leftTrigger()).whileTrue(
            runEnd(
                () -> intake.deployAndRunReverse(), // extend arm + roller backwards
                () -> intake.retractAndStop(),      // retract arm + stop roller on release
                intake                              // declare intake as the required subsystem
            )
        );

        // ===== SYSID ROUTINES (characterization — hold back/start + Y/X) =====
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
     * BLUE alliance:
     *   x < 3.978m → shoot directly into BLUE_HUB
     *   x >= 3.978m AND y > 4.035m → pass to BLUE_PASS_LEFT
     *   x >= 3.978m AND y <= 4.035m → pass to BLUE_PASS_RIGHT
     *
     * RED alliance:
     *   x > 12.563m → shoot directly into RED_HUB
     *   x <= 12.563m AND y < 4.035m → pass to RED_PASS_LEFT
     *   x <= 12.563m AND y >= 4.035m → pass to RED_PASS_RIGHT
     */
    private Translation2d getAimTarget() {
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        double x = robotPos.getX();
        double y = robotPos.getY();

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() &&
            alliance.get() == DriverStation.Alliance.Red;

        if (isRed) {
            if (x > Constants.RED_HUB_MIN_X) {
                return Constants.RED_HUB;
            } else if (y < Constants.PASS_SPLIT_Y) {
                return Constants.RED_PASS_LEFT;
            } else {
                return Constants.RED_PASS_RIGHT;
            }
        } else {
            if (x < Constants.BLUE_HUB_MAX_X) {
                return Constants.BLUE_HUB;
            } else if (y > Constants.PASS_SPLIT_Y) {
                return Constants.BLUE_PASS_LEFT;
            } else {
                return Constants.BLUE_PASS_RIGHT;
            }
        }
    }

    /**
     * Adjusts an aim target to compensate for the robot's current motion.
     *
     * When the robot is driving, the ball inherits its velocity. To compensate,
     * we aim slightly opposite to the direction of travel so the drift
     * carries the ball back to the real target.
     *
     * @param actualTarget The real field position we want to hit
     * @return A virtual aim point that accounts for robot motion
     */
    private Translation2d getLeadTarget(Translation2d actualTarget) {
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        double distance     = robotPosition.getDistance(actualTarget);
        double flightTimeSec = distance / Constants.BALL_SPEED_MPS;

        var robotSpeeds = drivetrain.getState().Speeds;
        var heading     = drivetrain.getState().Pose.getRotation();

        // Rotate robot-relative velocity into field-relative velocity
        double fieldVx = robotSpeeds.vxMetersPerSecond * heading.getCos()
                       - robotSpeeds.vyMetersPerSecond * heading.getSin();
        double fieldVy = robotSpeeds.vxMetersPerSecond * heading.getSin()
                       + robotSpeeds.vyMetersPerSecond * heading.getCos();

        Translation2d leadOffset = new Translation2d(
            fieldVx * flightTimeSec,
            fieldVy * flightTimeSec
        );
        return actualTarget.minus(leadOffset);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Vision getVision() {
        return vision;
    }
}