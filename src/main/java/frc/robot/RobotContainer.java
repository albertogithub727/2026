package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
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
    private final XboxController singleController = new XboxController(5);



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

    // Aim and Drive - HOLD TO AUTO-AIM
    private final JoystickButton aimButton = new JoystickButton(driver, XboxController.Button.kA.value);

    // Hood preset - set both actuators to 0.32
    private final JoystickButton hoodPresetButton = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Single Controller Buttons (port 5) */
    private final JoystickButton singleZeroGyro = new JoystickButton(singleController, XboxController.Button.kY.value);
    private final JoystickButton singleRobotCentric = new JoystickButton(singleController, XboxController.Button.kBack.value);
    private final JoystickButton singleReverseAll = new JoystickButton(singleController, XboxController.Button.kB.value);
    private final JoystickButton singleFeeder = new JoystickButton(singleController, XboxController.Button.kA.value);
    private final JoystickButton singleIntakeToggle = new JoystickButton(singleController, XboxController.Button.kX.value);
    private final JoystickButton singleExtendActuator = new JoystickButton(singleController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton singleRetractActuator = new JoystickButton(singleController, XboxController.Button.kRightBumper.value);

    /* Hood Presets */
    private final double[] hoodPresets = {0.0, 0.17, 0.32, 0.57};
    private int hoodPresetIndex = 0;

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Limelight limelight = new Limelight("limelight");
    private final Flywheel flywheel = new Flywheel();
    private final Intake intake = new Intake();
    private final Hood hood = new Hood();
    private final Shooter shooter = new Shooter();
    private final Climb climb = new Climb();

    /* SendableChooser for Autonomous Selection */
    private final SendableChooser<Command> chooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getRawAxis(translationAxis) - singleController.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis) - singleController.getRawAxis(strafeAxis)
                    + (driver.getLeftBumperButton() ? -0.2 : 0) + (driver.getRightBumperButton() ? 0.2 : 0)
                    + (singleController.getLeftBumperButton() ? -0.2 : 0) + (singleController.getRightBumperButton() ? 0.2 : 0),
                () -> -driver.getRawAxis(rotationAxis) - singleController.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean() || singleRobotCentric.getAsBoolean()
            )
        );

        limelight.setDefaultCommand(updateVisionCommand());

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
            new InstantCommand(() -> { if (!flywheel.isMotor1Extended()) flywheel.toggleMotor1Position(15.25); }));
        NamedCommands.registerCommand("IntakeUp",
            new InstantCommand(() -> { if (flywheel.isMotor1Extended()) flywheel.toggleMotor1Position(15.25); }));

        // Shooter
        NamedCommands.registerCommand("Shoot", new ShootCommand(shooter, flywheel));
        NamedCommands.registerCommand("ShooterSpinUp", new StartEndCommand(
            () -> shooter.setPercentOutput(Constants.Shooter.shooterSpeed), () -> shooter.stopAll(), shooter));
        NamedCommands.registerCommand("ShooterSpinUpRPM",
            new ShootCommand(shooter, flywheel, () -> shooter.setRPM(Constants.Shooter.shooterRPM)));
        NamedCommands.registerCommand("ShooterRPMOn", new InstantCommand(() -> shooter.setRPM(Constants.Shooter.autoShooterRPM)));
        final double[] feederStartPos = {0.0};
        NamedCommands.registerCommand("FeederOn", new FunctionalCommand(
            // init - save starting position, start feeder + track
            () -> {
                feederStartPos[0] = flywheel.getPosition1();
                shooter.runFeeder(Constants.Shooter.feederSpeed);
                flywheel.setVelocity2(-2000);
            },
            // execute - intake bop
            () -> {
                double elapsed = Timer.getFPGATimestamp() % (Constants.Shooter.intakeBopInterval * 2);
                double speed = elapsed < Constants.Shooter.intakeBopInterval
                    ? Constants.Shooter.intakeBopSpeed
                    : -Constants.Shooter.intakeBopSpeed;
                flywheel.setPercent1(speed);
            },
            // end - stop feeder, track, return intake to original position
            (interrupted) -> {
                shooter.runFeeder(0);
                flywheel.setVelocity2(0);
                flywheel.setPosition1(feederStartPos[0]);
            },
            // isFinished
            () -> false
        ));
        NamedCommands.registerCommand("ShooterOff", new InstantCommand(() -> {
            shooter.stopAll();
            flywheel.setVelocity2(0);
            flywheel.setPosition1(feederStartPos[0]); // return intake to original position
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

        // Climb
        NamedCommands.registerCommand("ClimbUp", new StartEndCommand(
            () -> climb.climbUp(), () -> climb.stop(), climb));
        NamedCommands.registerCommand("ClimbDown", new StartEndCommand(
            () -> climb.climbDown(), () -> climb.stop(), climb));
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        
        /* B Button - Reverse everything (hold to run) */
        feederButton.whileTrue(new StartEndCommand(
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

        /* Track Controls */
        feederButton2.onTrue(new InstantCommand(() -> flywheel.setVelocity2(-2000)));
        feederButton2.onFalse(new InstantCommand(() -> flywheel.setVelocity2(0)));

        // INTAKE DOWN - Driver D-Pad Up (hold for 30% power)
        new Trigger(() -> driver.getPOV() == 0)
            .whileTrue(new StartEndCommand(
                () -> flywheel.setPercent1(-0.2),
                () -> flywheel.setPercent1(0),
                flywheel
            ));

        // INTAKE UP - Driver D-Pad Down (hold for 30% power)
        new Trigger(() -> driver.getPOV() == 180)
            .whileTrue(new StartEndCommand(
                () -> flywheel.setPercent1(0.2),
                () -> flywheel.setPercent1(0),
                flywheel
            ));

        /* Aim and Drive - Hold A to auto-aim at hub while driving */
        aimButton.whileTrue(new AimAndDriveCommand(
            swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));

        /* Bumpers now used for strafing (added to default swerve command strafe supplier) */

        /* Intake Control - Left Trigger (65%) */
        new Trigger(() -> driver2.getLeftTriggerAxis() > 0.1)
            .whileTrue(new InstantCommand(() -> intake.intake(), intake))
            .onFalse(new InstantCommand(() -> intake.stop(), intake));

        /* Intake Control - D-Pad Left (100%) */
        new Trigger(() -> driver2.getPOV() == 270)
            .whileTrue(new StartEndCommand(
                () -> intake.setPercent(-1.0),
                () -> intake.stop(),
                intake
            ));

        /* Hood Preset - D-Pad Up (cycle up: 0 -> 0.32 -> 1.0) */
        new Trigger(() -> driver2.getPOV() == 0)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex < hoodPresets.length - 1) {
                    hoodPresetIndex++;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        /* Hood Preset - D-Pad Down (cycle down: 1.0 -> 0.32 -> 0) */
        new Trigger(() -> driver2.getPOV() == 180)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex > 0) {
                    hoodPresetIndex--;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        /* Prepare Shot + Auto-Aim - D-Pad Right (hold to aim, spin up shooter, position hood, then feed after delay) */
        new Trigger(() -> driver2.getPOV() == 90)
            .whileTrue(Commands.parallel(
                new AimAndDriveCommand(swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis)),
                new PrepareShotCommand(shooter, hood, swerve::getPose),
                Commands.waitSeconds(Constants.Shooter.feederDelay)
                    .andThen(new StartEndCommand(
                        () -> {
                            shooter.runFeeder(Constants.Shooter.feederSpeed);
                            flywheel.setVelocity2(-2000);
                        },
                        () -> {
                            shooter.runFeeder(0);
                            flywheel.setVelocity2(0);
                        }
                    ))
            ));

        /* Shooter Control - Right Trigger (speed based on hood preset) */
        new Trigger(() -> driver2.getRightTriggerAxis() > 0.1)
            .whileTrue(new ShootCommand(shooter, flywheel, this::setShooterForPreset));

        /* Climb Control - Driver 1 Triggers (hold to move) */
        new Trigger(() -> driver.getRightTriggerAxis() > 0.1)
            .whileTrue(new StartEndCommand(
                () -> climb.climbUp(),
                () -> climb.stop(),
                climb
            ));

        new Trigger(() -> driver.getLeftTriggerAxis() > 0.1)
            .whileTrue(new StartEndCommand(
                () -> climb.climbDown(),
                () -> climb.stop(),
                climb
            ));

        /* ===== SINGLE CONTROLLER (port 5) ===== */

        /* Zero Gyro - Y */
        singleZeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

        /* Reverse All (feeder/unjam) - B (hold) */
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

        /* Track/Feeder - A (hold) */
        singleFeeder.onTrue(new InstantCommand(() -> flywheel.setVelocity2(-2000)));
        singleFeeder.onFalse(new InstantCommand(() -> flywheel.setVelocity2(0)));

        /* Intake Deploy Toggle - X */
        singleIntakeToggle.onTrue(new InstantCommand(() -> flywheel.toggleMotor1Position(-15.25)));

        /* Bumpers now used for strafing (added to default swerve command strafe supplier) */

        /* Intake - Left Trigger (half power) */
        new Trigger(() -> singleController.getLeftTriggerAxis() > 0.1)
            .whileTrue(new InstantCommand(() -> intake.setPercent(-0.375), intake))
            .onFalse(new InstantCommand(() -> intake.stop(), intake));

        /* Shoot - Right Trigger (speed based on hood preset) */
        new Trigger(() -> singleController.getRightTriggerAxis() > 0.1)
            .whileTrue(new ShootCommand(shooter, flywheel, this::setShooterForPreset));

        /* Climb Up - D-Pad Up (hold) */
        new Trigger(() -> singleController.getPOV() == 0)
            .whileTrue(new StartEndCommand(
                () -> climb.climbUp(),
                () -> climb.stop(),
                climb
            ));

        /* Climb Down - D-Pad Down (hold) */
        new Trigger(() -> singleController.getPOV() == 180)
            .whileTrue(new StartEndCommand(
                () -> climb.climbDown(),
                () -> climb.stop(),
                climb
            ));

        /* Hood Preset Up - D-Pad Right */
        new Trigger(() -> singleController.getPOV() == 90)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex < hoodPresets.length - 1) {
                    hoodPresetIndex++;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));

        /* Hood Preset Down - D-Pad Left */
        new Trigger(() -> singleController.getPOV() == 270)
            .onTrue(new InstantCommand(() -> {
                if (hoodPresetIndex > 0) {
                    hoodPresetIndex--;
                }
                hood.setPosition(hoodPresets[hoodPresetIndex]);
                SmartDashboard.putNumber("Hood Preset", hoodPresetIndex);
            }, hood));
    }
    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getPose();
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose,
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        }).ignoringDisable(true);
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

    private void setShooterForPreset() {
        switch (hoodPresetIndex) {
            case 0:
                shooter.setPercentOutput(Constants.Shooter.shooterSpeedHoodDown);
                SmartDashboard.putString("Shooter Power", Constants.Shooter.shooterSpeedHoodDown + " duty cycle");
                break;
            case 1:
                shooter.setRPM(3500);
                SmartDashboard.putString("Shooter Power", "3000 RPM");
                break;
            case 2:
                shooter.setRPM(Constants.Shooter.shooterRPM);
                SmartDashboard.putString("Shooter Power", Constants.Shooter.shooterRPM + " RPM");
                break;
            case 3:
                shooter.setRPM(3900);
                SmartDashboard.putString("Shooter Power", "3900 RPM");
                break;
            default:
                shooter.setRPM(Constants.Shooter.shooterRPM);
                SmartDashboard.putString("Shooter Power", Constants.Shooter.shooterRPM + " RPM");
                break;
        }
    }

    public Hood getHood() {
        return hood;
    }
}