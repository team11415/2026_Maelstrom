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
import edu.wpi.first.networktables.DoublePublisher;
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

    // ---- Swerve requests ----
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    point = new SwerveRequest.PointWheelsAt();

    // Auto-rotates to a target heading while the driver still controls translation.
    // Think of it like lane-keep assist: you steer around the field, the robot
    // handles pointing at the goal.
    private final SwerveRequest.FieldCentricFacingAngle aimDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // =========================================================================
    // CONTROLLERS
    // =========================================================================
    // Port 0 = driver controller   (driving + shooting)
    // Port 1 = operator controller (toggles + fine-tuning)
    // =========================================================================
    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // =========================================================================
    // SUBSYSTEMS
    // =========================================================================
    // IMPORTANT: spindexer, shooter, leds, and intake MUST be declared before
    // dashboardTelemetry because DashboardTelemetry's constructor takes shooter
    // and spindexer as parameters. Java initializes fields top-to-bottom, so
    // those subsystems must exist before dashboardTelemetry is created.
    // =========================================================================
    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();

    private final Spindexer spindexer = new Spindexer();
    private final Shooter   shooter   = new Shooter();
    private final LEDs      leds      = new LEDs();
    private final Intake    intake    = new Intake();

    @SuppressWarnings("unused")
    private final DashboardTelemetry dashboardTelemetry =
        new DashboardTelemetry(drivetrain, shooter, spindexer);

    private final Vision vision = new Vision(
        () -> drivetrain.getState().Pose,
        (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs),
        () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);

    // =========================================================================
    // CONTROL FLAG NT ENTRIES
    // =========================================================================
    // These are on/off toggles visible in Elastic as Toggle Switch widgets.
    // They all live under "ControlFlags" in NetworkTables so they're grouped.
    //
    // AimAssistEnabled:
    //   ON  → right trigger makes the robot auto-rotate to face the target
    //   OFF → driver aims manually with the right stick
    //   Toggled by: Operator Y button
    //
    // CameraAEnabled (limelight-a individually):
    //   ON  → limelight-a feeds pose estimates to the drivetrain
    //   OFF → limelight-a is ignored (still powered, just not used)
    //   Toggled by: Operator A button
    //
    // CameraBEnabled (limelight-b individually):
    //   ON  → limelight-b feeds pose estimates to the drivetrain
    //   OFF → limelight-b is ignored (still powered, just not used)
    //   Toggled by: Operator B button
    //
    // Note: Vision.java also has a master VisionEnabled switch that cuts
    // BOTH cameras at once. That master switch can be toggled directly in
    // Elastic at ControlFlags/VisionEnabled if needed, but there is no
    // gamepad button wired to it — use it as a dashboard-only emergency stop.
    // =========================================================================
    private final NetworkTableEntry aimAssistEnabledEntry;
    private final NetworkTableEntry cameraAEnabledEntry; // limelight-a individual toggle
    private final NetworkTableEntry cameraBEnabledEntry; // limelight-b individual toggle

    // =========================================================================
    // OPERATOR FINE-TUNE OFFSETS
    // =========================================================================
    //
    // rpsOffset — flywheel speed trim
    // ────────────────────────────────
    // Added on top of the distance-based SHOOTER_SPEED_MAP lookup result.
    // Think of it like the trim knob on a speaker: the lookup table gives you
    // the base volume (speed) for the distance, and rpsOffset nudges it up or
    // down. If you're consistently undershooting, press D-pad Up a few times.
    //
    //   Operator D-pad Up   → +1 RPS per press
    //   Operator D-pad Down → -1 RPS per press
    //   Operator Start      → reset to 0
    //   Safety clamp        → limited to ±30 RPS from the lookup value
    //
    //
    // aimAngleOffsetDeg — heading direction trim
    // ─────────────────────────────────────────
    // Adjusts the angle the robot auto-rotates to when aim assist is ON.
    // Useful if the robot consistently aims a few degrees left or right of the
    // goal — dial it in mid-match without stopping. Think of it like adjusting
    // a rifle's iron sights: small nudges to walk the shots onto the target.
    //
    //   Operator D-pad Right → +1 degree per press (aim more clockwise)
    //   Operator D-pad Left  → -1 degree per press (aim more counterclockwise)
    //   Operator Start       → reset to 0
    //   Driver Start + LB    → ALSO resets to 0 (see field reset binding below)
    //   Safety clamp         → limited to ±45 degrees
    //
    //
    // WHY does aimAngleOffsetDeg reset on a field position reset?
    //   The angle offset corrects for a consistent aiming error observed from
    //   a particular spot on the field. When you reset your field position
    //   (Driver Start + Left Bumper), you're moving to a new known reference
    //   point. The old angle correction may no longer apply and could make
    //   things worse, so we clear it for a fresh start at the new position.
    //   The RPS offset does NOT reset on field position reset — speed errors
    //   are usually mechanical or environmental, not position-dependent.
    // =========================================================================
    private double rpsOffset         = 0.0;
    private double aimAngleOffsetDeg = 0.0;

    // Publishers so these values appear as numbers in Elastic / AdvantageScope.
    // In Elastic, add these keys as "Number Display" widgets:
    //   Operator/RPSOffset       → current speed trim in ± RPS
    //   Operator/AimAngleOffset  → current heading trim in ± degrees
    //
    // Other useful widgets to add:
    //   Shooter/TargetRPS_Active          → flywheel's actual current target
    //   ControlFlags/AimAssistEnabled     → aim assist on/off (true/false)
    //   ControlFlags/CameraAEnabled       → limelight-a on/off (true/false)
    //   ControlFlags/CameraBEnabled       → limelight-b on/off (true/false)
    //   ControlFlags/VisionEnabled        → master camera kill switch (dashboard only)
    private final DoublePublisher rpsOffsetPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Operator/RPSOffset").publish();
    private final DoublePublisher aimAngleOffsetPub = NetworkTableInstance.getDefault()
        .getDoubleTopic("Operator/AimAngleOffset").publish();

    private final SendableChooser<Command> autoChooser;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public RobotContainer() {

        // ---- Set up control flag NT entries ----
        // All entries live in "ControlFlags" so all toggles are grouped.
        aimAssistEnabledEntry = NetworkTableInstance.getDefault()
            .getTable("ControlFlags").getEntry("AimAssistEnabled");
        aimAssistEnabledEntry.setDefaultBoolean(true); // starts ON

        // Individual camera entries — entry names match exactly what
        // Vision.java uses internally so both read the same NT keys.
        // Vision.java sets both defaults to true in its own constructor
        // (which runs when the vision field above is initialized).
        // We just need references to those same entries here so operator
        // buttons can toggle them.
        cameraAEnabledEntry = NetworkTableInstance.getDefault()
            .getTable("ControlFlags").getEntry("CameraAEnabled");
        cameraBEnabledEntry = NetworkTableInstance.getDefault()
            .getTable("ControlFlags").getEntry("CameraBEnabled");

        // Publish initial offset values so Elastic shows 0 instead of
        // a blank widget before the operator first presses a button.
        rpsOffsetPub.set(0.0);
        aimAngleOffsetPub.set(0.0);

        configureBindings();

        // =========================================================================
        // NAMED COMMANDS FOR PATHPLANNER AUTO
        // =========================================================================

        // "runShooter" — spin up flywheel, wait until at speed, feed the ball.
        //
        // Uses waitUntil(isAtTargetSpeed) instead of a fixed delay.
        // Old approach: "wait exactly 0.25 seconds then fire regardless."
        // New approach: "wait until the flywheel is actually ready (up to 1.5s)."
        // This handles cases where the battery is low (slower spin-up) or the
        // flywheel was already warm (faster spin-up) automatically.
        NamedCommands.registerCommand("runShooter",
            Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.runShooter(); // reads Tuning/TargetRPS from dashboard
                    leds.setShooting(true);
                }, shooter),
                Commands.waitUntil(shooter::isAtTargetSpeed).withTimeout(1.5),
                runEnd(
                    () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                    () -> { spindexer.stopAll(); shooter.stopShooter();
                            leds.setShooting(false); },
                    spindexer, shooter
                )
            ).withTimeout(4.0)
        );

        // "runIntake" — deploy arm + spin roller forward, retract when done.
        NamedCommands.registerCommand("runIntake",
            runEnd(
                () -> { intake.deployAndRun(); leds.setIntaking(true); },
                () -> { intake.retractAndStop(); leds.setIntaking(false); },
                intake
            )
        );

        NamedCommands.registerCommand("runWait2",  new WaitCommand(2.0));
        NamedCommands.registerCommand("runWait4",  new WaitCommand(4.0));
        NamedCommands.registerCommand("stopAll",
            Commands.runOnce(() -> {
                spindexer.stopAll();
                shooter.stopShooter();
            }, spindexer, shooter)
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // =========================================================================
    // BUTTON BINDINGS
    // =========================================================================
    private void configureBindings() {

        // ===== DEFAULT DRIVE COMMAND =====
        // Left stick XY = translate, Right stick X = rotate.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // Let drive motors rest in neutral while the robot is disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Heading PID for aim-assist auto-rotation.
        // kP=7.0: for every 1 radian of heading error, apply 7 rad/s correction.
        // enableContinuousInput: tells the PID that 179° and -179° are neighbors,
        // so it always takes the shortest path around the circle.
        aimDrive.HeadingController.setPID(7.0, 0.0, 0.0);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);


        // =========================================================================
        // DRIVER BINDINGS
        // =========================================================================

        // ---- A: X-lock wheels ----
        // Crosses the wheels so the robot resists being pushed.
        // Good for defending or holding position at end-game.
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // ---- B: Point wheels at left stick direction ----
        // Steers the wheels without moving. Useful for lining up quickly.
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(
                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // ---- Left trigger: Run intake ----
        // Hold → extend arm + spin roller forward.
        // Release → retract arm + stop roller automatically.
        driver.leftTrigger(0.5).whileTrue(
            runEnd(
                () -> intake.deployAndRun(),
                () -> intake.retractAndStop(),
                intake
            )
        );

        // ---- Right trigger: Aim-assist + shoot ----
        //
        // Two things happen in PARALLEL when you hold the right trigger:
        //
        //   ARM 1 (drive):
        //     If aim assist ON  → robot auto-rotates to face the target.
        //                         aimAngleOffsetDeg is applied here.
        //     If aim assist OFF → driver rotates manually with right stick.
        //     Either way, driver controls translation (left stick) freely.
        //
        //   ARM 2 (shoot sequence):
        //     1. Look up flywheel speed for current distance, add rpsOffset
        //     2. Wait until flywheel reaches that speed (up to 1.5 seconds)
        //     3. Feed ball through spindexer + yeeter
        //     4. Release trigger → everything stops, LEDs reset
        //
        // All values (aim assist flag, rpsOffset, aimAngleOffsetDeg) are read
        // every 20ms inside the lambdas, so the operator can adjust them while
        // the driver holds the trigger — no need to release and re-press.
        driver.rightTrigger(0.5)
            .whileTrue(
                Commands.parallel(

                    // ---- ARM 1: Drive with auto-rotate or manual ----
                    drivetrain.applyRequest(() -> {

                        boolean aimAssistOn = aimAssistEnabledEntry.getBoolean(true);

                        if (!aimAssistOn) {
                            // Aim assist OFF: driver controls all rotation.
                            // Shooter still fires in ARM 2 — they just aim manually.
                            return drive
                                .withVelocityX(-driver.getLeftY() * MaxSpeed)
                                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                .withRotationalRate(-driver.getRightX() * MaxAngularRate);
                        }

                        // Aim assist ON: calculate the angle from robot to target.
                        // getLeadTarget() adjusts for the robot's current velocity
                        // (like a quarterback throwing ahead of a moving receiver).
                        Translation2d aimTarget     = getLeadTarget(getAimTarget());
                        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
                        Translation2d robotToTarget = aimTarget.minus(robotPosition);

                        // atan2 = the compass bearing from robot to target, in radians.
                        double angleToTargetRad = Math.atan2(
                            robotToTarget.getY(), robotToTarget.getX());

                        // Two offsets are subtracted from the raw bearing:
                        //
                        //   SHOOTER_ANGLE_OFFSET_DEG = fixed 90° mechanical offset
                        //   because the shooter faces the LEFT side of the robot,
                        //   not the front. This never changes.
                        //
                        //   aimAngleOffsetDeg = operator's live fine-tune adjustment.
                        //   Positive values shift aim counterclockwise (robot turns
                        //   right slightly). If shots consistently land to the LEFT
                        //   of target, add a positive offset. If to the RIGHT, use
                        //   a negative offset. Adjust with Operator D-pad Left/Right.
                        double desiredHeadingRad = angleToTargetRad
                            - Math.toRadians(Constants.SHOOTER_ANGLE_OFFSET_DEG)
                            - Math.toRadians(aimAngleOffsetDeg);

                        return aimDrive
                            .withVelocityX(-driver.getLeftY() * MaxSpeed)
                            .withVelocityY(-driver.getLeftX() * MaxSpeed)
                            .withTargetDirection(new Rotation2d(desiredHeadingRad));
                    }),

                    // ---- ARM 2: Shoot sequence ----
                    Commands.sequence(
                        // Step 1: Spin up flywheel to distance-based speed + operator trim.
                        Commands.runOnce(() -> {
                            Translation2d realTarget    = getAimTarget();
                            Translation2d robotPosition =
                                drivetrain.getState().Pose.getTranslation();
                            double distance  = robotPosition.getDistance(realTarget);

                            // Lookup returns the base speed for this distance.
                            // rpsOffset is the operator's trim — e.g. if the table
                            // returns 70 RPS and operator pressed D-pad Up 3 times,
                            // we fire at 73 RPS.
                            double autoSpeed =
                                Constants.SHOOTER_SPEED_MAP.get(distance) + rpsOffset;
                            shooter.runShooterAtSpeed(autoSpeed);
                            leds.setShooting(true);
                        }, shooter),

                        // Step 2: Wait for flywheel to reach target speed.
                        // withTimeout(1.5) ensures we still fire even if something
                        // is wrong — better a slightly slow shot than no shot.
                        Commands.waitUntil(shooter::isAtTargetSpeed).withTimeout(1.5),

                        // Step 3: Feed the ball. Runs until trigger is released,
                        // at which point the end action fires automatically.
                        runEnd(
                            () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                            () -> { shooter.stopShooter(); spindexer.stopAll();
                                    leds.setShooting(false); },
                            shooter, spindexer
                        )
                    )
                )
            );

        // ---- Left bumper: Reset field-centric heading ----
        // Re-sets "forward" to whatever direction the robot is currently facing.
        // Use this if field-relative drive feels rotated the wrong way.
        driver.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ---- Start + Left bumper: Reset pose to known field position ----
        //
        // Drive to a pre-agreed spot on the field, then press Start + Left Bumper.
        // The robot's X,Y snaps to Constants.BLUE/RED_RESET_POSITION while keeping
        // the current gyro heading unchanged.
        //
        // Also clears aimAngleOffsetDeg: the angle correction was tuned for your
        // previous location. At a new known position, it may no longer apply and
        // could make aiming worse, so we clear it automatically for a fresh start.
        // The RPS offset is NOT cleared — speed errors are not position-dependent.
        driver.start().and(driver.leftBumper()).onTrue(
            drivetrain.runOnce(() -> {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() &&
                    alliance.get() == DriverStation.Alliance.Red;

                Translation2d knownPosition = isRed
                    ? Constants.RED_RESET_POSITION
                    : Constants.BLUE_RESET_POSITION;

                Rotation2d currentHeading =
                    drivetrain.getState().Pose.getRotation();
                drivetrain.resetPose(
                    new Pose2d(knownPosition, currentHeading));

                // Clear aim angle offset — fresh position, fresh start.
                aimAngleOffsetDeg = 0.0;
                aimAngleOffsetPub.set(0.0);
            })
        );

        // ---- Back + right trigger: Reverse spindexer + yeeter (jam clearing) ----
        // Hold both to run the carousel and yeeter backwards.
        // Release either button to stop both motors.
        driver.back().and(driver.rightTrigger()).whileTrue(
            runEnd(
                () -> { spindexer.runSpindexerReverse(); spindexer.runYeeterReverse(); },
                () -> spindexer.stopAll(),
                spindexer
            )
        );

        // ---- Back + left trigger: Reverse intake (eject) ----
        // Hold both to extend the arm and spin the roller backwards.
        // Release either button to retract the arm and stop the roller.
        driver.back().and(driver.leftTrigger()).whileTrue(
            runEnd(
                () -> intake.deployAndRunReverse(),
                () -> intake.retractAndStop(),
                intake
            )
        );

        // ---- SysId characterization routines ----
        // Only used during PID tuning sessions — not during matches.
        driver.back().and(driver.y()).whileTrue(
            drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(
            drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(
            drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(
            drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ---- Telemetry ----
        drivetrain.registerTelemetry(logger::telemeterize);


        // =========================================================================
        // OPERATOR CONTROLLER BINDINGS (port 1)
        // =========================================================================
        //
        // BUTTON MAP:
        // ┌─────────────────┬───────────────────────────────────────────────────┐
        // │ Y               │ Toggle aim assist ON/OFF                          │
        // │ A               │ Toggle limelight-a (Camera A) ON/OFF              │
        // │ B               │ Toggle limelight-b (Camera B) ON/OFF              │
        // │ D-pad Up        │ Shooter speed +1 RPS (above distance lookup)      │
        // │ D-pad Down      │ Shooter speed -1 RPS                              │
        // │ D-pad Right     │ Aim angle +1 degree (clockwise correction)        │
        // │ D-pad Left      │ Aim angle -1 degree (counterclockwise correction) │
        // │ Start           │ Reset BOTH offsets to zero                        │
        // └─────────────────┴───────────────────────────────────────────────────┘
        //
        // ELASTIC DASHBOARD WIDGETS TO ADD:
        //   Operator/RPSOffset            → Number Display  (current speed trim ±RPS)
        //   Operator/AimAngleOffset       → Number Display  (current angle trim ±deg)
        //   Shooter/TargetRPS_Active      → Number Display  (actual flywheel target)
        //   ControlFlags/AimAssistEnabled → Toggle Switch   (aim assist on/off)
        //   ControlFlags/CameraAEnabled   → Toggle Switch   (limelight-a on/off)
        //   ControlFlags/CameraBEnabled   → Toggle Switch   (limelight-b on/off)
        //   ControlFlags/VisionEnabled    → Toggle Switch   (master kill switch,
        //                                                    dashboard-only — no button)
        // =========================================================================

        // ---- Operator Y: Toggle aim assist ----
        // Flips the value each press: ON→OFF or OFF→ON.
        // You can also flip it directly in Elastic — the button and the widget
        // both read and write the same NetworkTables entry.
        operator.y().onTrue(Commands.runOnce(() -> {
            boolean current = aimAssistEnabledEntry.getBoolean(true);
            aimAssistEnabledEntry.setBoolean(!current);
        }));

        // ---- Operator A: Toggle Camera A (limelight-a) ----
        // Flips limelight-a ON or OFF without affecting limelight-b.
        // When OFF, limelight-a is still powered and running — the robot
        // simply stops using its pose estimates. Useful if limelight-a has
        // a bad view of the field (blocked by a defense robot, etc).
        // Watch the state at ControlFlags/CameraAEnabled in Elastic.
        operator.a().onTrue(Commands.runOnce(() -> {
            boolean current = cameraAEnabledEntry.getBoolean(true);
            cameraAEnabledEntry.setBoolean(!current);
        }));

        // ---- Operator B: Toggle Camera B (limelight-b) ----
        // Flips limelight-b ON or OFF without affecting limelight-a.
        // Same behavior as Camera A above, but for the second camera.
        // Watch the state at ControlFlags/CameraBEnabled in Elastic.
        operator.b().onTrue(Commands.runOnce(() -> {
            boolean current = cameraBEnabledEntry.getBoolean(true);
            cameraBEnabledEntry.setBoolean(!current);
        }));

        // ---- Operator D-pad Up: Flywheel +1 RPS ----
        // Each press adds 1 RPS to whatever the distance lookup returns.
        // If you're consistently undershooting, press this a few times.
        // Clamped at +30 RPS to prevent runaway speed commands.
        operator.povUp().onTrue(Commands.runOnce(() -> {
            rpsOffset = Math.min(rpsOffset + 1.0, 30.0);
            rpsOffsetPub.set(rpsOffset);
        }));

        // ---- Operator D-pad Down: Flywheel -1 RPS ----
        // Each press subtracts 1 RPS from the lookup result.
        // If you're consistently overshooting (ball going too far), press this.
        // Clamped at -30 RPS for safety.
        operator.povDown().onTrue(Commands.runOnce(() -> {
            rpsOffset = Math.max(rpsOffset - 1.0, -30.0);
            rpsOffsetPub.set(rpsOffset);
        }));

        // ---- Operator D-pad Right: Aim angle +1 degree ----
        // Shifts the auto-aim heading 1 degree clockwise per press.
        // Use this when shots consistently land to the LEFT of the target —
        // nudging clockwise rotates the robot slightly to close that gap.
        // Clamped at +45 degrees.
        operator.povRight().onTrue(Commands.runOnce(() -> {
            aimAngleOffsetDeg = Math.min(aimAngleOffsetDeg + 1.0, 45.0);
            aimAngleOffsetPub.set(aimAngleOffsetDeg);
        }));

        // ---- Operator D-pad Left: Aim angle -1 degree ----
        // Shifts the auto-aim heading 1 degree counterclockwise per press.
        // Use this when shots consistently land to the RIGHT of the target.
        // Clamped at -45 degrees.
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            aimAngleOffsetDeg = Math.max(aimAngleOffsetDeg - 1.0, -45.0);
            aimAngleOffsetPub.set(aimAngleOffsetDeg);
        }));

        // ---- Operator Start: Reset all offsets to zero ----
        // Clears BOTH rpsOffset and aimAngleOffsetDeg back to zero in one press.
        // Use this if adjustments made things worse and you want a clean slate.
        operator.start().onTrue(Commands.runOnce(() -> {
            rpsOffset         = 0.0;
            aimAngleOffsetDeg = 0.0;
            rpsOffsetPub.set(0.0);
            aimAngleOffsetPub.set(0.0);
        }));
    }

    // =========================================================================
    // HELPER: GET BEST AIM TARGET
    // =========================================================================
    // Returns the field position to aim at based on robot location and alliance.
    //
    // BLUE alliance:
    //   x < BLUE_HUB_MAX_X (3.978m)  → shoot at BLUE_HUB directly
    //   x ≥ 3.978, y > PASS_SPLIT_Y  → pass to BLUE_PASS_LEFT (upper field)
    //   x ≥ 3.978, y ≤ PASS_SPLIT_Y  → pass to BLUE_PASS_RIGHT (lower field)
    //
    // RED alliance:
    //   x > RED_HUB_MIN_X (12.563m)  → shoot at RED_HUB directly
    //   x ≤ 12.563, y < PASS_SPLIT_Y → pass to RED_PASS_LEFT
    //   x ≤ 12.563, y ≥ PASS_SPLIT_Y → pass to RED_PASS_RIGHT
    // =========================================================================
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

    // =========================================================================
    // HELPER: LEAD TARGET (motion compensation)
    // =========================================================================
    // When the robot is moving, the ball inherits that velocity. If we aim
    // directly at the real target, the ball drifts away from it during flight.
    //
    // This method calculates a virtual aim point shifted opposite to the
    // direction of travel, so the ball's drift carries it back onto the target.
    // Analogy: a quarterback doesn't throw to where the receiver IS — they
    // throw to where the receiver WILL BE when the ball arrives.
    // =========================================================================
    private Translation2d getLeadTarget(Translation2d actualTarget) {
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        double distance      = robotPosition.getDistance(actualTarget);
        double flightTimeSec = distance / Constants.BALL_SPEED_MPS;

        var robotSpeeds = drivetrain.getState().Speeds;
        var heading     = drivetrain.getState().Pose.getRotation();

        // Convert robot-relative velocity to field-relative velocity.
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