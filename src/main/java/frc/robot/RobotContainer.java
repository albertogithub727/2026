package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.MoveActuator;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterTuningCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController driver2 = new XboxController(1);
   // private final XboxController tuningController = new XboxController(2);

    // private final XboxController singleController = new XboxController(5);



    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton feederButton = new JoystickButton(driver2, XboxController.Button.kB.value);
    private final JoystickButton feederButton2 = new JoystickButton(driver2, XboxController.Button.kA.value);
    // Linear Actuator controls - HOLD TO MOVE
    private final JoystickButton extendActuator = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton retractActuator = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorRightBumper = new JoystickButton(driver2, XboxController.Button.kRightBumper.value);

    // Aim and Drive - HOLD TO AUTO-AIM
    private final JoystickButton aimButton = new JoystickButton(driver, XboxController.Button.kA.value);

    // Aim and Drive (Fast Oscillation) - HOLD TO AUTO-AIM with faster intake oscillation
    private final JoystickButton aimButtonFast = new JoystickButton(driver, XboxController.Button.kB.value);

    // Hood preset - set both actuators to 0.32
    private final JoystickButton hoodPresetButton = new JoystickButton(driver, XboxController.Button.kX.value);

    // Auto-Shoot - Start button (hold to auto-aim + distance-based RPM/hood + feed)
    private final JoystickButton autoShootButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton autoShootButton2 = new JoystickButton(driver2, XboxController.Button.kStart.value);

    /* Single Controller Buttons (port 5) */
    // private final JoystickButton singleZeroGyro = new JoystickButton(singleController, XboxController.Button.kY.value);
    // private final JoystickButton singleRobotCentric = new JoystickButton(singleController, XboxController.Button.kBack.value);
    // private final JoystickButton singleReverseAll = new JoystickButton(singleController, XboxController.Button.kB.value);
    // private final JoystickButton singleFeeder = new JoystickButton(singleController, XboxController.Button.kA.value);
    // private final JoystickButton singleIntakeToggle = new JoystickButton(singleController, XboxController.Button.kX.value);
    // private final JoystickButton singleExtendActuator = new JoystickButton(singleController, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton singleRetractActuator = new JoystickButton(singleController, XboxController.Button.kRightBumper.value);
    // private final JoystickButton singleAutoShoot = new JoystickButton(singleController, XboxController.Button.kStart.value);

    /* Hood Presets (in rotations) */
    private final double[] hoodPresets = {0.0, -2, -3, -3.5};
    private int hoodPresetIndex = 0;
    private double currentShooterRPM = 0; // Track current RPM for boost calculation
    private boolean hoodStalled = false;
    private final Timer hoodStallTimer = new Timer();
    private final Timer strafeOscillateTimer = new Timer();
    private boolean strafeOscillateActive = false;

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Limelight limelight = new Limelight("limelight");
    private final Limelight limelightLeft = new Limelight("limelight-left");
    private final Flywheel flywheel = new Flywheel();
    private final Intake intake = new Intake();
    private final Hood hood = new Hood();
    private final Shooter shooter = new Shooter();
    /* SendableChooser for Autonomous Selection */
    private final SendableChooser<Command> chooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        SignalLogger.enableAutoLogging(false);
     swerve.setDefaultCommand(
    new TeleopSwerve(
        swerve,
        () -> driver.getRawAxis(translationAxis),   // removed the negative
        () -> driver.getRawAxis(strafeAxis)          // removed the negative
            + (driver.getLeftBumperButton() ? -0.2 : 0) + (driver.getRightBumperButton() ? 0.2 : 0)
            + getStrafeOscillation(),
        () -> -driver.getRawAxis(rotationAxis),
        () -> robotCentric.getAsBoolean()
    )
);


        limelight.setDefaultCommand(updateVisionCommand());
        hood.setDefaultCommand(new FunctionalCommand(
            // init: reset stall flag, start timer, and start commanding 0
            () -> {
                hoodStalled = false;
                hoodStallTimer.restart();
                hood.setPosition(0);
            },
            // execute: once stalled, stay stopped; skip stall check for first 0.5s
            () -> {
                if (hoodStalled) {
                    return;
                }
                if (hoodStallTimer.hasElapsed(0.5)
                        && Math.abs(hood.getStatorCurrent()) > 40
                        && Math.abs(hood.getVelocity()) < 0.5) {
                    hoodStalled = true;
                    hood.stop();
                } else {
                    hood.setPosition(0);
                }
            },
            // end: stop motor when command is interrupted
            interrupted -> hood.stop(),
            // isFinished: never
            () -> false,
            hood
        ));

        configureButtonBindings();
        registerNamedCommands();

        FollowPathCommand.warmupCommand().schedule();
        chooser = AutoBuilder.buildAutoChooser("main");
        SmartDashboard.putData("Auto Mode", chooser);
    }

    private void registerNamedCommands() {
        // Intake
        NamedCommands.registerCommand("IntakeOn", new StartEndCommand(
            () -> intake.intake(), () -> intake.stop(), intake));
        NamedCommands.registerCommand("IntakeOff", new InstantCommand(() -> intake.stop(), intake));
        NamedCommands.registerCommand("IntakeFull", new StartEndCommand(
            () -> intake.setPercent(-1.0), () -> intake.stop(), intake));
        NamedCommands.registerCommand("Outtake", new StartEndCommand(
            () -> intake.outtake(), () -> intake.stop(), intake));

        // Intake deploy (up/down)
        NamedCommands.registerCommand("IntakeDown",
            new InstantCommand(() -> { if (!flywheel.isMotor1Extended()) flywheel.toggleMotor1Position(17.5); }));
        NamedCommands.registerCommand("IntakeUp",
            new InstantCommand(() -> { if (flywheel.isMotor1Extended()) flywheel.toggleMotor1Position(15.25); }));

        // Shooter
        NamedCommands.registerCommand("Shoot", new ShootCommand(shooter, flywheel, intake));
        NamedCommands.registerCommand("ShooterSpinUp", new StartEndCommand(
            () -> shooter.setPercentOutput(Constants.Shooter.shooterSpeed), () -> shooter.stopAll(), shooter));
        NamedCommands.registerCommand("ShooterSpinUpRPM",
            new ShootCommand(shooter, flywheel, intake, () -> shooter.setRPM(Constants.Shooter.shooterRPM)));
        NamedCommands.registerCommand("ShooterRPMOn", Commands.sequence(
            // Start at boosted RPM + intake
            new InstantCommand(() -> {
                shooter.setRPM(Constants.Shooter.autoShooterRPM + 200);
                intake.intake();
            }),
            // Wait 1 second with boost (ramp-up)
            Commands.waitSeconds(1.0),
            // Drop to base RPM
            new InstantCommand(() -> shooter.setRPM(Constants.Shooter.autoShooterRPM))
        ));
        NamedCommands.registerCommand("ShooterHood15", Commands.sequence(
            // Set hood and start at boosted RPM
            new InstantCommand(() -> {
                hood.setPosition(0.11);  // Converted from 19mm to rotations
                shooter.setRPM(3575);  // 3400 + 200 RPM boost
            }),
            // Wait 1 second with boost
            Commands.waitSeconds(1.0),
            // Drop to base RPM
            new InstantCommand(() -> shooter.setRPM(3250))
        ));
        final double[] feederStartPos = {0.0};
        NamedCommands.registerCommand("FeederOn", new FunctionalCommand(
            // init - save starting position, start feeder + track
            () -> {
                feederStartPos[0] = flywheel.getPosition1();
                shooter.runFeeder(Constants.Shooter.feederSpeed);
                flywheel.setVelocity2(-2000);
                intake.setPercent(-.25f);
            },
            // execute - no-op
            () -> {},
            // end - stop feeder + track
            (interrupted) -> {
                shooter.runFeeder(0);
                flywheel.setVelocity2(0);
            },
            // isFinished
            () -> false
        ));
        NamedCommands.registerCommand("ShooterOff", new InstantCommand(() -> {
            shooter.stopAll();
            flywheel.setVelocity2(0);
            flywheel.setPosition1(feederStartPos[0]);
        }));

        // Prepare shot (auto hood + shooter based on distance)
        NamedCommands.registerCommand("PrepareShot",
            new PrepareShotCommand(shooter, hood, swerve::getPose));

        // Aim and drive (auto-rotate toward hub)
        NamedCommands.registerCommand("AimAtHub", new AimAndDriveCommand(swerve));

        // Aim + prepare shot combined
        NamedCommands.registerCommand("AimAndPrepareShot", Commands.parallel(
            new AimAndDriveCommand(swerve),
            new PrepareShotCommand(shooter, hood, swerve::getPose)));

        // Aim + shoot with feeder and intake oscillation (same as A button)
        NamedCommands.registerCommand("AimAndShoot", Commands.parallel(
            new AimAndDriveCommand(swerve),
            aimAndShootSequence()
        ));

        // Auto-shoot (same as start button) - aim + prepare + feed
        NamedCommands.registerCommand("AutoShoot", Commands.parallel(
            new AimAndDriveCommand(swerve),
            new PrepareShotCommand(shooter, hood, swerve::getPose),
            Commands.sequence(
                Commands.waitSeconds(Constants.Shooter.feederDelay),
                feedAndBopCommand()
            )
        ));

        // Reverse everything (unjam)
        NamedCommands.registerCommand("Reverse", new StartEndCommand(
            () -> {
                shooter.setPercentOutput(-Constants.Shooter.shooterSpeed);
                shooter.runFeeder(-Constants.Shooter.feederSpeed);
                intake.setPercent(0.65);
                flywheel.setVelocity2(2000);
            },
            () -> {
                shooter.stopAll();
                intake.stop();
                flywheel.setVelocity2(0);
            },
            shooter, intake));

        // Hood
        NamedCommands.registerCommand("HoodExtend", new MoveActuator(hood, true));
        NamedCommands.registerCommand("HoodRetract", new MoveActuator(hood, false));

    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

        /* B Button - Aim and Shoot (Fast Oscillation) - same as A but faster intake oscillation */
        aimButtonFast.whileTrue(
            Commands.parallel(
                new AimAndDriveCommand(swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis)),
                aimAndShootSequenceFast()
            )
        );

        /* Set hood to 0.7 when B button is released */
        aimButtonFast.onFalse(new InstantCommand(() -> hood.setPosition(0.7), hood));

        /* Driver2 B Button - Reverse everything (hold to run) */
        feederButton.whileTrue(new StartEndCommand(
            () -> {
                shooter.setPercentOutput(-Constants.Shooter.shooterSpeed);
                shooter.runFeeder(-Constants.Shooter.feederSpeed);
                intake.setPercent(-0.65);
                flywheel.setVelocity2(2000);
            },
            () -> {
                shooter.stopAll();
                intake.stop();
                flywheel.setVelocity2(0);
            },
            shooter, intake
        ));

        /*new JoystickButton(tuningController, XboxController.Button.kBack.value)
    .whileTrue(new ShooterTuningCommand(shooter, hood, swerve, tuningController));*/

        /* Driver2 Y Button - Intake down fast (75% power) */
        new JoystickButton(driver2, XboxController.Button.kY.value)
            .whileTrue(new StartEndCommand(
                () -> flywheel.setPercent1(-0.75),
                () -> flywheel.setPercent1(0),
                flywheel
            ));

        /* Driver2 X Button - Hold to run track motor (motor ID 10) only */
        new JoystickButton(driver2, XboxController.Button.kX.value)
            .whileTrue(new StartEndCommand(
                () -> flywheel.setVelocity2(-2000),
                () -> flywheel.setVelocity2(0),
                flywheel
            ));

        /* Driver2 A Button - Maximum range shot (0.80 shooter, -3.2 hood) with aim and shoot sequence */
        feederButton2.whileTrue(
            Commands.parallel(
                // Fixed shooter and hood (max range shot)
                new StartEndCommand(
                    () -> {
                        shooter.setPercentOutput(0.80);
                        hood.setPosition(-3.2);
                    },
                    () -> {
                        shooter.stopAll();
                        hood.setPosition(0.7);
                    },
                    shooter, hood
                ),
                // Same feeder and intake sequence as aimButton
                Commands.sequence(
                    Commands.waitSeconds(.75),
                    new StartEndCommand(
                        () -> {
                            shooter.runFeeder(0.90);
                            flywheel.setPercent2(-0.90);
                        },
                        () -> {
                            shooter.runFeeder(0);
                            flywheel.setPercent2(0);
                        }
                    )
                )
            )
        );

        /* Aim and Drive + Shoot - Hold A to auto-aim at hub, distance-based RPM/hood, then feed */
        aimButton.whileTrue(
            Commands.parallel(
                new AimAndDriveCommand(swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis)),
                aimAndShootSequence()
            )
        );

        /* Set hood to 0.7 when A button is released */
        aimButton.onFalse(new InstantCommand(() -> hood.setPosition(0.7), hood));

        /* Bumpers now used for strafing (added to default swerve command strafe supplier) */

        /* Driver D-Pad Up - Next hood preset */
        new Trigger(() -> driver.getPOV() == 0)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex < hoodPresets.length - 1) {
                    hoodPresetIndex++;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        /* Driver D-Pad Down - Previous hood preset */
        new Trigger(() -> driver.getPOV() == 180)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex > 0) {
                    hoodPresetIndex--;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        /* Intake Control - Left Trigger (both motors) */
        new Trigger(() -> driver2.getLeftTriggerAxis() > 0.1)
            .whileTrue(new InstantCommand(() -> intake.intake(), intake))
            .onFalse(new InstantCommand(() -> intake.stop(), intake));

        /* INTAKE DOWN - D-Pad Left (hold for 20% power) */
        new Trigger(() -> driver2.getPOV() == 270)
            .whileTrue(new StartEndCommand(
                () -> flywheel.setPercent1(-0.2),
                () -> flywheel.setPercent1(0),
                flywheel
            ));

        /* Hood Preset Up - D-Pad Up */
       /* Hood Up - D-Pad Up (hold to move) */
/* Hood Up - D-Pad Up (hold to move) */
new Trigger(() -> driver2.getPOV() == 0)
    .whileTrue(new StartEndCommand(
        () -> hood.setPercent(-0.2),
        () -> hood.setPercent(0),
        hood
    ));

/* Hood Down - D-Pad Down (hold to move) */
new Trigger(() -> driver2.getPOV() == 180)
    .whileTrue(new StartEndCommand(
        () -> hood.setPercent(0.2),
        () -> hood.setPercent(0),
        hood
    ));

    /* Driver2 Back Button - Zero hood encoder */
new JoystickButton(driver2, XboxController.Button.kBack.value)
    .onTrue(new InstantCommand(() -> hood.zeroEncoder(), hood));

        /* INTAKE UP - D-Pad Right (hold for 20% power) */
        new Trigger(() -> driver2.getPOV() == 90)
            .whileTrue(new StartEndCommand(
                () -> flywheel.setPercent1(0.2),
                () -> flywheel.setPercent1(0),
                flywheel
            ));

        /* ============================================================
         * AUTO-SHOOT - START BUTTON (Driver 1 or Driver 2)
         * Hold to: auto-aim at hub + auto-adjust RPM/hood + feed
         * Uses Limelight pose estimation for distance calculation
         * ============================================================ */

        // Driver 1 Start: auto-aim + auto-shoot (driver 1 can still translate)
        autoShootButton.whileTrue(
            Commands.parallel(
                // Auto-aim at hub while allowing driver translation
                new AimAndDriveCommand(swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis)),
                // Auto-adjust RPM and hood based on Limelight distance
                new PrepareShotCommand(shooter, hood, swerve::getPose),
                // After spin-up delay, start feeding
                Commands.sequence(
                    Commands.waitSeconds(Constants.Shooter.feederDelay),
                    feedAndBopCommand()
                )
            )
        );

        // Driver 2 Start: same auto-shoot (driver 1 still controls translation)
        autoShootButton2.whileTrue(
            Commands.parallel(
                new AimAndDriveCommand(swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis)),
                new PrepareShotCommand(shooter, hood, swerve::getPose),
                Commands.sequence(
                    Commands.waitSeconds(Constants.Shooter.feederDelay),
                    feedAndBopCommand()
                )
            )
        );

        /* Shooter Control - Right Trigger (speed based on hood preset, shooter motors only) */
        new Trigger(() -> driver2.getRightTriggerAxis() > 0.1)
            .whileTrue(new StartEndCommand(
                () -> setShooterForPreset(),
                () -> shooter.stop(),
                shooter
            ));

        /* Driver2 Right Bumper - Shooter at 50%, feeders+track after .25s */
        operatorRightBumper.whileTrue(
            Commands.parallel(
                // Start shooter motors at 50% immediately
                new StartEndCommand(
                    () -> shooter.setPercentOutput(0.5),
                    () -> shooter.stopAll(),
                    shooter
                ),
                // Wait .25 seconds, then start feeders + track
                Commands.sequence(
                    Commands.waitSeconds(.25),
                    new StartEndCommand(
                        () -> {
                            shooter.runFeeder(0.25);
                            flywheel.setPercent2(-0.75);
                        },
                        () -> {
                            shooter.runFeeder(0);
                            flywheel.setPercent2(0);
                        }
                    )
                )
            )
        );

        /* ===== SINGLE CONTROLLER (port 5) ===== */
        /*
        singleZeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

        singleReverseAll.whileTrue(new StartEndCommand(
            () -> {
                shooter.setPercentOutput(-Constants.Shooter.shooterSpeed);
                shooter.runFeeder(-Constants.Shooter.feederSpeed);
                intake.setPercent(0.65);
                flywheel.setVelocity2(2000);
            },
            () -> {
                shooter.stopAll();
                intake.stop();
                flywheel.setVelocity2(0);
            },
            shooter, intake
        ));

        singleFeeder.onTrue(new InstantCommand(() -> flywheel.setVelocity2(-2000)));
        singleFeeder.onFalse(new InstantCommand(() -> flywheel.setVelocity2(0)));

        singleIntakeToggle.onTrue(new InstantCommand(() -> flywheel.toggleMotor1Position(-15.25)));

        new Trigger(() -> singleController.getLeftTriggerAxis() > 0.1)
            .whileTrue(new InstantCommand(() -> intake.setPercent(-0.375), intake))
            .onFalse(new InstantCommand(() -> intake.stop(), intake));

        new Trigger(() -> singleController.getRightTriggerAxis() > 0.1)
            .whileTrue(new ShootCommand(shooter, flywheel, intake, this::setShooterForPreset));

        new Trigger(() -> singleController.getPOV() == 90)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex < hoodPresets.length - 1) {
                    hoodPresetIndex++;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        new Trigger(() -> singleController.getPOV() == 270)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex > 0) {
                    hoodPresetIndex--;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        singleAutoShoot.whileTrue(
            Commands.parallel(
                new AimAndDriveCommand(swerve,
                    () -> -singleController.getRawAxis(translationAxis),
                    () -> -singleController.getRawAxis(strafeAxis)),
                new PrepareShotCommand(shooter, hood, swerve::getPose),
                Commands.sequence(
                    Commands.waitSeconds(Constants.Shooter.feederDelay),
                    feedAndBopCommand()
                )
            )
        );
        */
    }

    /**
     * Creates a command that runs the feeder and track.
     * Used by the auto-shoot sequence after the shooter has spun up.
     * Does NOT declare shooter/flywheel as requirements so it can run
     * in parallel with PrepareShotCommand.
     */
    /**
     * Creates the shared aim-and-shoot sequence: prepare shot, delayed feeder,
     * and intake arm oscillation. Used by both the A button (teleop) and the
     * "AimAndShoot" named command (autonomous).
     */
    private Command aimAndShootSequence() {
        return Commands.parallel(
            new PrepareShotCommand(shooter, hood, swerve::getPose),
            Commands.sequence(
                Commands.waitSeconds(.75),
                new StartEndCommand(
                    () -> {
                        shooter.runFeeder(0.90);
                        flywheel.setPercent2(-0.90);
                    },
                    () -> {
                        shooter.runFeeder(0);
                        flywheel.setPercent2(0);
                    }
                )
            )
        );
    }

    /**
     * Creates aim-and-shoot sequence with INSTANT intake oscillation.
     * Same as aimAndShootSequence() but intake arm starts oscillating IMMEDIATELY
     * instead of waiting 1.7 seconds. Used by the B button.
     */
    private Command aimAndShootSequenceFast() {
        return Commands.parallel(
            new PrepareShotCommand(shooter, hood, swerve::getPose),
            Commands.sequence(
                Commands.waitSeconds(.25),
                new StartEndCommand(
                    () -> {
                        shooter.runFeeder(0.90);
                        flywheel.setPercent2(-0.90);
                    },
                    () -> {
                        shooter.runFeeder(0);
                        flywheel.setPercent2(0);
                    }
                )
            )
        );
    }

    private Command feedAndBopCommand() {
        return new FunctionalCommand(
            // init: start feeder + track
            () -> {
                shooter.runFeeder(Constants.Shooter.feederSpeed);
                flywheel.setVelocity2(-2000);
            },
            // execute - no-op
            () -> {},
            // end: stop everything
            (interrupted) -> {
                shooter.runFeeder(0);
                flywheel.setVelocity2(0);
            },
            // never finishes - runs until parent command group is cancelled
            () -> false
        );
    }

    private double getStrafeOscillation() {
        if (!strafeOscillateActive) return 0.0;
        double time = strafeOscillateTimer.get();
        double interval = Constants.Shooter.shootStrafeInterval;
        return ((int)(time / interval) % 2 == 0)
            ? Constants.Shooter.shootStrafeSpeed
            : -Constants.Shooter.shootStrafeSpeed;
    }

    /** True after odometry has been seeded with the first valid vision pose. */
    private boolean visionInitialized = false;

    private void processVisionMeasurement(Limelight ll) {
        final Pose2d currentRobotPose = swerve.getPose();

        // Pass current angular velocity so Limelight can filter fast-spin estimates
        ll.setAngularVelocity(swerve.getAngularVelocityDegS());

        ll.getMeasurement(currentRobotPose).ifPresent(m -> {
            // On the first valid vision reading, hard-reset odometry so the
            // pose estimator starts at the correct field position instead of (0,0).
            if (!visionInitialized) {
                swerve.resetPose(m.poseEstimate.pose);
                visionInitialized = true;
            }

            swerve.addVisionMeasurement(
                m.poseEstimate.pose,
                m.poseEstimate.timestampSeconds,
                m.standardDeviations
            );
        });
    }

    private Command updateVisionCommand() {
        return Commands.run(() -> {
            processVisionMeasurement(limelight);
            processVisionMeasurement(limelightLeft);
        }, limelight, limelightLeft).ignoringDisable(true);
    }

    /**
     * Use this to pass the autonomous command to the main Robot class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
    
    public Swerve getSwerve() {
        return swerve;
    }

    public Intake getIntake() {
        return intake;
    }

    private Command createShootCommandForPreset() {
        return Commands.deferredProxy(() -> {
            setShooterForPreset();
            // Hood preset 2 gets reduced RPM boost (100 instead of 200)
            double rpmBoost = (hoodPresetIndex == 2) ? 100 : 200;
            // Hood preset 2 gets longer feeder delay (2.5s instead of 2s)
            double feederDelay = (hoodPresetIndex == 2) ? 2.5 : Constants.Shooter.feederDelay;
            return new ShootCommand(shooter, flywheel, intake, () -> {}, currentShooterRPM, rpmBoost, feederDelay);
        });
    }

    private void setShooterForPreset() {
        switch (hoodPresetIndex) {
            case 0:
                shooter.setPercentOutput(Constants.Shooter.shooterSpeedHoodDown);
                currentShooterRPM = 0; // Duty cycle mode, no RPM boost
                SmartDashboard.putString("Shooter Power", Constants.Shooter.shooterSpeedHoodDown + " duty cycle");
                break;
            case 1:
                currentShooterRPM = 2810;
                shooter.setRPM(currentShooterRPM);
                SmartDashboard.putString("Shooter Power", "2875 RPM");
                break;
            case 2:
                currentShooterRPM = Constants.Shooter.shooterRPM;
                shooter.setRPM(currentShooterRPM);
                SmartDashboard.putString("Shooter Power", Constants.Shooter.shooterRPM + " RPM");
                break;
            case 3:
                currentShooterRPM = 4700;
                shooter.setRPM(currentShooterRPM);
                SmartDashboard.putString("Shooter Power", "3200 RPM");
                break;
            default:
                currentShooterRPM = Constants.Shooter.shooterRPM;
                shooter.setRPM(currentShooterRPM);
                SmartDashboard.putString("Shooter Power", Constants.Shooter.shooterRPM + " RPM");
                break;
        }
    }

    public Hood getHood() {
        return hood;
    }
}