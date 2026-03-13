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
import edu.wpi.first.networktables.DoublePublisher;        // NEW: for operator telemetry
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
    // Port 1 = operator controller (NEW: toggles + fine-tuning)
    // =========================================================================
    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1); // NEW

    // =========================================================================
    // SUBSYSTEMS
    // =========================================================================
    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();

    // Spindexer and Shooter MUST be declared before DashboardTelemetry
    // because DashboardTelemetry's constructor now takes them as parameters.
    // Java initializes fields top-to-bottom, so they have to exist first.
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
    // These are the on/off toggles visible in Elastic as Toggle Switch widgets.
    // They all live under "ControlFlags" in NetworkTables.
    //
    // AimAssistEnabled:
    //   ON  → right trigger makes the robot auto-rotate to the target
    //   OFF → driver aims manually with the right stick
    //
    // VisionEnabled (also read by Vision.java):
    //   ON  → Limelight cameras update the robot's position estimate
    //   OFF → robot uses only wheel odometry + gyro (cameras still running,
    //          just being ignored)
    // =========================================================================
    private final NetworkTableEntry aimAssistEnabledEntry;
    private final NetworkTableEntry visionEnabledEntry; // NEW: for operator camera toggle

    // =========================================================================
    // OPERATOR FINE-TUNE OFFSETS  (NEW)
    // =========================================================================
    //
    // rpsOffset — flywheel speed trim
    // ────────────────────────────────
    // Added on top of the distance-based SHOOTER_SPEED_MAP lookup result.
    // Think of it like the "trim" knob on a speaker: the lookup table gives you
    // the base volume (speed) for the distance, and rpsOffset nudges it up or
    // down. If you're consistently undershooting, press D-pad Up a few times.
    //
    //   Operator D-pad Up   → +1 RPS per press
    //   Operator D-pad Down → -1 RPS per press
    //   Operator Start      → reset to 0
    //   Safety clamp        → limited to ±30 RPS above/below the lookup value
    //
    //
    // aimAngleOffsetDeg — heading direction trim
    // ─────────────────────────────────────────
    // Adjusts the angle the robot auto-rotates to when aim assist is ON.
    // Useful if the robot consistently aims a few degrees left or right of the
    // goal — dial it in mid-match without stopping. Think of it like adjusting
    // a gun's iron sights: small nudges to walk the shots onto the target.
    //
    //   Operator D-pad Right → +1 degree per press
    //   Operator D-pad Left  → -1 degree per press
    //   Operator Start       → reset to 0
    //   Driver Start + LB    → ALSO resets to 0 (see below for why)
    //   Safety clamp         → limited to ±45 degrees
    //
    //
    // WHY does aimAngleOffsetDeg reset on a field position reset?
    //   The angle offset corrects for a consistent aiming error you observed
    //   from a particular spot on the field. When you reset your field position
    //   (Start + Left Bumper), you're moving to a new known reference point.
    //   The old angle correction may no longer apply and could make things
    //   worse, so we clear it for a fresh start at the new position.
    //   The RPS offset does NOT reset on field position reset — speed errors
    //   are usually mechanical/environmental, not position-dependent.
    // =========================================================================
    private double rpsOffset         = 0.0;
    private double aimAngleOffsetDeg = 0.0;

    // Publishers so these values appear as numbers in Elastic / AdvantageScope.
    // In Elastic, search for these keys and add as "Number Display" widgets:
    //   Operator/RPSOffset       → current speed trim in ± RPS
    //   Operator/AimAngleOffset  → current heading trim in ± degrees
    //
    // Also useful to add to your dashboard:
    //   Shooter/TargetRPS_Active         → what speed the flywheel is actually
    //                                       targeting this moment (already published
    //                                       by Shooter.java — includes rpsOffset)
    //   ControlFlags/AimAssistEnabled    → aim assist on/off (true/false)
    //   ControlFlags/VisionEnabled       → cameras on/off (true/false)
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
        // Both entries live in "ControlFlags" so all on/off toggles are grouped.
        aimAssistEnabledEntry = NetworkTableInstance.getDefault()
            .getTable("ControlFlags").getEntry("AimAssistEnabled");
        aimAssistEnabledEntry.setDefaultBoolean(true); // starts ON

        // Vision.java sets this default to true in its constructor (which runs
        // before this body because Vision is a field initialized above).
        // We just need a reference to the same entry so the operator can toggle it.
        visionEnabledEntry = NetworkTableInstance.getDefault()
            .getTable("ControlFlags").getEntry("VisionEnabled");

        // Publish initial offset values (0) so Elastic shows a number instead
        // of a blank widget before the operator first presses a button.
        rpsOffsetPub.set(0.0);
        aimAngleOffsetPub.set(0.0);

        configureBindings();

        // =========================================================================
        // NAMED COMMANDS FOR PATHPLANNER AUTO
        // =========================================================================

        // "runShooter" — spin up flywheel, wait until at speed, then feed the ball.
        //
        // IMPROVEMENT: now uses waitUntil(isAtTargetSpeed) instead of a fixed 0.25s
        // delay. Old code was like "wait exactly 3 minutes for a pot to boil."
        // New code is "wait until the pot is actually boiling (up to 5 minutes)."
        NamedCommands.registerCommand("runShooter",
            Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.runShooter(); // uses Tuning/TargetRPS from dashboard
                    leds.setShooting(true);
                }, shooter),
                // Wait until flywheel reaches target speed, or give up after 1.5 seconds.
                // This is more reliable than a fixed wait because spin-up time varies
                // based on battery voltage and how recently the shooter was last used.
                Commands.waitUntil(shooter::isAtTargetSpeed).withTimeout(1.5),
                runEnd(
                    () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                    () -> { spindexer.stopAll(); shooter.stopShooter(); leds.setShooting(false); },
                    spindexer, shooter
                )
            ).withTimeout(4.0)
        );

        // "runIntake" — deploy arm + spin roller forward, retract when done.
        NamedCommands.registerCommand("runIntake",
            runEnd(
                () -> {
                    intake.deployAndRun();
                    leds.setIntaking(true);
                },
                () -> {
                    intake.retractAndStop();
                    leds.setIntaking(false);
                },
                intake
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

    // =========================================================================
    // BUTTON BINDINGS
    // =========================================================================
    private void configureBindings() {

        // ===== DEFAULT DRIVE COMMAND =====
        // Left stick XY = translate, Right stick X = rotate
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // Idle while disabled — drive motors rest in neutral.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Heading PID for aim-assist auto-rotation.
        // kP=7.0: for every 1 radian of heading error, apply 7 rad/s of correction.
        // enableContinuousInput: teaches the PID that 179° and -179° are neighbors.
        aimDrive.HeadingController.setPID(7.0, 0.0, 0.0);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // ===== DRIVER: A — X-LOCK WHEELS =====
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // ===== DRIVER: B — POINT WHEELS AT LEFT STICK DIRECTION =====
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // ===== DRIVER: LEFT TRIGGER — RUN INTAKE =====
        // Hold: extend arm + spin roller forward.
        // Release: retract arm + stop roller automatically.
        driver.leftTrigger(0.5).whileTrue(
            runEnd(
                () -> intake.deployAndRun(),
                () -> intake.retractAndStop(),
                intake
            )
        );

        // ===== DRIVER: RIGHT TRIGGER — AIM-ASSIST + SHOOT =====
        //
        // Two things happen in PARALLEL when you hold the right trigger:
        //
        //   ARM 1 (drive control):
        //     If aim assist ON  → robot auto-rotates to face the target.
        //                         aimAngleOffsetDeg is applied here.
        //     If aim assist OFF → driver controls rotation with right stick.
        //     Either way, driver controls translation (left stick) freely.
        //
        //   ARM 2 (shoot sequence):
        //     1. Look up flywheel speed for current distance, add rpsOffset
        //     2. Wait until flywheel is at speed (up to 1.5 seconds max)
        //     3. Feed ball through spindexer + yeeter
        //     4. Release trigger → shooter stops, spindexer stops, LEDs reset
        //
        // All values (aim assist flag, rpsOffset, aimAngleOffsetDeg) are read
        // every 20ms INSIDE the lambdas, so the operator can adjust them
        // while the driver holds the trigger — no need to release and re-press.
        driver.rightTrigger(0.5)
            .whileTrue(
                Commands.parallel(

                    // ---- ARM 1: Drive control (auto-rotate or manual) ----
                    drivetrain.applyRequest(() -> {

                        boolean aimAssistOn = aimAssistEnabledEntry.getBoolean(true);

                        if (!aimAssistOn) {
                            // Aim assist OFF: hand full rotation control to the driver.
                            // The shooter still fires in ARM 2; they just aim manually.
                            return drive
                                .withVelocityX(-driver.getLeftY() * MaxSpeed)
                                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                .withRotationalRate(-driver.getRightX() * MaxAngularRate);
                        }

                        // Aim assist ON: calculate the angle from robot to target.
                        // getLeadTarget() shifts the aim point to compensate for
                        // the robot's current velocity (like leading a moving target).
                        Translation2d aimTarget     = getLeadTarget(getAimTarget());
                        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
                        Translation2d robotToTarget = aimTarget.minus(robotPosition);

                        // atan2 = compass bearing from the robot to the target, in radians.
                        double angleToTargetRad = Math.atan2(
                            robotToTarget.getY(), robotToTarget.getX());

                        // Two offsets are subtracted here:
                        //   SHOOTER_ANGLE_OFFSET_DEG = fixed 90° mechanical offset because
                        //   the shooter faces the LEFT side of the robot (not the front).
                        //
                        //   aimAngleOffsetDeg = operator's live fine-tune adjustment.
                        //   Positive values rotate aim counterclockwise (robot turns right
                        //   to compensate). If you're consistently overshooting left,
                        //   add a positive offset. If overshooting right, use negative.
                        //   Test on the field and adjust with D-pad Left/Right.
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
                        // Step 1: Spin up flywheel to the right speed for this distance.
                        Commands.runOnce(() -> {
                            Translation2d realTarget    = getAimTarget();
                            Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
                            double distance  = robotPosition.getDistance(realTarget);

                            // Look up base speed for this distance from the table in Constants,
                            // then add the operator's trim offset.
                            // Example: table returns 70 RPS, operator set +3 → fires at 73 RPS.
                            double autoSpeed = Constants.SHOOTER_SPEED_MAP.get(distance) + rpsOffset;
                            shooter.runShooterAtSpeed(autoSpeed);
                            leds.setShooting(true);
                        }, shooter),

                        // Step 2: Wait until the flywheel is actually at target speed.
                        // Gives up after 1.5 seconds so we still fire even if the
                        // shooter is having trouble reaching speed (better to fire
                        // slightly slow than not fire at all).
                        Commands.waitUntil(shooter::isAtTargetSpeed).withTimeout(1.5),

                        // Step 3: Feed the ball. Runs until trigger is released.
                        runEnd(
                            () -> { spindexer.runSpindexer(); spindexer.runYeeter(); },
                            () -> { shooter.stopShooter(); spindexer.stopAll();
                                    leds.setShooting(false); },
                            shooter, spindexer
                        )
                    )
                )
            );

        // ===== DRIVER: LEFT BUMPER — RESET FIELD-CENTRIC HEADING =====
        // Re-sets which direction "forward" is to the robot's current facing.
        // Use this if field-relative drive feels rotated the wrong way.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // =========================================================================
        // DRIVER: START + LEFT BUMPER — RESET POSE TO KNOWN FIELD POSITION
        // =========================================================================
        // Drive to a pre-agreed spot on the field, then hold Start + Left Bumper.
        // The robot's X,Y position snaps to Constants.BLUE/RED_RESET_POSITION.
        // Rotation is NOT changed — the gyro is still trusted for heading.
        //
        // Also clears aimAngleOffsetDeg: you're at a new reference position,
        // so the old angle correction may no longer apply. See the offset
        // field comments at the top of this file for full explanation.
        // =========================================================================
        driver.start().and(driver.leftBumper()).onTrue(
            drivetrain.runOnce(() -> {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() &&
                    alliance.get() == DriverStation.Alliance.Red;

                Translation2d knownPosition = isRed
                    ? Constants.RED_RESET_POSITION
                    : Constants.BLUE_RESET_POSITION;

                Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
                drivetrain.resetPose(new Pose2d(knownPosition, currentHeading));

                // Clear the aim angle offset — fresh position, fresh start.
                aimAngleOffsetDeg = 0.0;
                aimAngleOffsetPub.set(0.0);
            })
        );

        // =========================================================================
        // DRIVER: BACK + RIGHT TRIGGER — REVERSE SPINDEXER + YEETER
        // =========================================================================
        // Hold both to run spindexer and yeeter backwards (jam clearing).
        // Release either button to stop both motors.
        // =========================================================================
        driver.back().and(driver.rightTrigger()).whileTrue(
            runEnd(
                () -> { spindexer.runSpindexerReverse(); spindexer.runYeeterReverse(); },
                () -> spindexer.stopAll(),
                spindexer
            )
        );

        // =========================================================================
        // DRIVER: BACK + LEFT TRIGGER — REVERSE INTAKE (eject)
        // =========================================================================
        // Hold both to extend arm and spin roller backwards (eject game piece).
        // Release either button to retract arm and stop roller automatically.
        // =========================================================================
        driver.back().and(driver.leftTrigger()).whileTrue(
            runEnd(
                () -> intake.deployAndRunReverse(),
                () -> intake.retractAndStop(),
                intake
            )
        );

        // ===== SYSID CHARACTERIZATION ROUTINES =====
        // Only used during PID tuning — not during matches.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ===== TELEMETRY =====
        drivetrain.registerTelemetry(logger::telemeterize);


        // =========================================================================
        // OPERATOR CONTROLLER BINDINGS  (NEW — port 1)
        // =========================================================================
        //
        // BUTTON MAP:
        // ┌─────────────────┬───────────────────────────────────────────────────┐
        // │ Y               │ Toggle aim assist ON/OFF                          │
        // │ B               │ Toggle cameras (vision) ON/OFF                    │
        // │ D-pad Up        │ Shooter speed +1 RPS (above distance lookup)      │
        // │ D-pad Down      │ Shooter speed -1 RPS                              │
        // │ D-pad Right     │ Aim angle +1 degree (clockwise correction)        │
        // │ D-pad Left      │ Aim angle -1 degree (counterclockwise correction) │
        // │ Start           │ Reset BOTH offsets to zero                        │
        // └─────────────────┴───────────────────────────────────────────────────┘
        //
        // ELASTIC DASHBOARD WIDGETS TO ADD:
        //   Operator/RPSOffset           → Number Display  (shows ±RPS trim)
        //   Operator/AimAngleOffset      → Number Display  (shows ±degree trim)
        //   Shooter/TargetRPS_Active     → Number Display  (actual flywheel target)
        //   ControlFlags/AimAssistEnabled → Toggle Switch  (aim assist state)
        //   ControlFlags/VisionEnabled    → Toggle Switch  (camera state)
        // =========================================================================

        // ---- OPERATOR Y: Toggle aim assist ----
        // Flips the value each press. You can also flip it in Elastic directly.
        operator.y().onTrue(Commands.runOnce(() -> {
            boolean current = aimAssistEnabledEntry.getBoolean(true);
            aimAssistEnabledEntry.setBoolean(!current);
        }));

        // ---- OPERATOR B: Toggle cameras (master vision switch) ----
        // Turning OFF tells the robot to ignore all Limelight data until turned
        // back ON. Cameras stay powered; their pose updates just aren't used.
        operator.b().onTrue(Commands.runOnce(() -> {
            boolean current = visionEnabledEntry.getBoolean(true);
            visionEnabledEntry.setBoolean(!current);
        }));

        // ---- OPERATOR D-PAD UP: Flywheel +1 RPS ----
        // Each press adds 1 RPS to whatever the distance lookup returns.
        // Clamped at +30 RPS for safety (prevents runaway speed commands).
        operator.povUp().onTrue(Commands.runOnce(() -> {
            rpsOffset = Math.min(rpsOffset + 1.0, 30.0);
            rpsOffsetPub.set(rpsOffset);
        }));

        // ---- OPERATOR D-PAD DOWN: Flywheel -1 RPS ----
        // Each press subtracts 1 RPS from the lookup result.
        // Clamped at -30 RPS for safety.
        operator.povDown().onTrue(Commands.runOnce(() -> {
            rpsOffset = Math.max(rpsOffset - 1.0, -30.0);
            rpsOffsetPub.set(rpsOffset);
        }));

        // ---- OPERATOR D-PAD RIGHT: Aim angle +1 degree ----
        // Shifts the auto-aim heading 1 degree clockwise.
        // Use when shots consistently land to the LEFT of target.
        // Clamped at +45 degrees.
        operator.povRight().onTrue(Commands.runOnce(() -> {
            aimAngleOffsetDeg = Math.min(aimAngleOffsetDeg + 1.0, 45.0);
            aimAngleOffsetPub.set(aimAngleOffsetDeg);
        }));

        // ---- OPERATOR D-PAD LEFT: Aim angle -1 degree ----
        // Shifts the auto-aim heading 1 degree counterclockwise.
        // Use when shots consistently land to the RIGHT of target.
        // Clamped at -45 degrees.
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            aimAngleOffsetDeg = Math.max(aimAngleOffsetDeg - 1.0, -45.0);
            aimAngleOffsetPub.set(aimAngleOffsetDeg);
        }));

        // ---- OPERATOR START: Reset all offsets ----
        // Clears both rpsOffset and aimAngleOffsetDeg back to zero in one press.
        // Use this if you dialed in adjustments that made things worse.
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
    // Returns the field position to aim at based on where the robot is
    // and which alliance we're on.
    //
    // BLUE alliance:
    //   x < 3.978m → shoot directly at BLUE_HUB
    //   x >= 3.978, y > 4.035 → pass to BLUE_PASS_LEFT (upper field)
    //   x >= 3.978, y <= 4.035 → pass to BLUE_PASS_RIGHT (lower field)
    //
    // RED alliance:
    //   x > 12.563m → shoot directly at RED_HUB
    //   x <= 12.563, y < 4.035 → pass to RED_PASS_LEFT
    //   x <= 12.563, y >= 4.035 → pass to RED_PASS_RIGHT
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
    // When the robot is moving, the ball inherits that velocity. If we aim at
    // the real target, the ball drifts past it during flight.
    //
    // This shifts the virtual aim point opposite to the direction of travel
    // so the ball's drift carries it BACK onto the real target.
    // Think of a quarterback throwing ahead of a receiver — you throw to where
    // they'll be, not where they are now.
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