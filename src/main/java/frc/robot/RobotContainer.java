package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController driver2 = new XboxController(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton feederButton = new JoystickButton(driver2, XboxController.Button.kB.value);
    private final JoystickButton feederButton2 = new JoystickButton(driver2, XboxController.Button.kA.value);
    private final JoystickButton toggleMotor2Button = new JoystickButton(driver2, XboxController.Button.kX.value);
    
    // Linear Actuator controls - HOLD TO MOVE
    private final JoystickButton extendActuator = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton retractActuator = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    // Aim and Drive - HOLD TO AUTO-AIM
    private final JoystickButton aimButton = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Flywheel Flywheel = new Flywheel();
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
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        configureButtonBindings();
        registerNamedCommands();

        FollowPathCommand.warmupCommand().schedule();
        chooser = AutoBuilder.buildAutoChooser("New Auto");
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
            new InstantCommand(() -> { if (!Flywheel.isMotor1Extended()) Flywheel.toggleMotor1Position(-15.25); }));
        NamedCommands.registerCommand("IntakeUp",
            new InstantCommand(() -> { if (Flywheel.isMotor1Extended()) Flywheel.toggleMotor1Position(-15.25); }));

        // Shooter
        NamedCommands.registerCommand("Shoot", new ShootCommand(shooter, Flywheel));
        NamedCommands.registerCommand("ShooterSpinUp", new StartEndCommand(
            () -> shooter.setPercentOutput(Constants.Shooter.shooterSpeed), () -> shooter.stopAll(), shooter));
        NamedCommands.registerCommand("ShooterOff", new InstantCommand(() -> shooter.stopAll(), shooter));

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
                Flywheel.setVelocity2(2000);
            },
            () -> {
                shooter.stopAll();
                intake.stop();
                Flywheel.setVelocity2(0);
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
                Flywheel.setVelocity2(2000);
            },
            () -> {
                shooter.stopAll();
                intake.stop();
                Flywheel.setVelocity2(0);
            },
            shooter, intake
        ));

        /* Track Controls */
        feederButton2.onTrue(new InstantCommand(() -> Flywheel.setVelocity2(-2000)));
        feederButton2.onFalse(new InstantCommand(() -> Flywheel.setVelocity2(0)));

        // INTAKE DOWN OR UP //
        toggleMotor2Button.onTrue(new InstantCommand(() -> Flywheel.toggleMotor1Position(-15.25)));

        /* Aim and Drive - Hold A to auto-aim at hub while driving */
        aimButton.whileTrue(new AimAndDriveCommand(
            swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));

        /* Linear Actuator Control - HOLD TO MOVE CONTINUOUSLY */
        
        // LEFT BUMPER: Hold to extend gradually
        // Releases automatically when button is released
        extendActuator.whileTrue(new MoveActuator(hood, true));

        // RIGHT BUMPER: Hold to retract gradually
        // Releases automatically when button is released
        retractActuator.whileTrue(new MoveActuator(hood, false));

        /* Intake Control - Left Trigger (65%) */
        new Trigger(() -> driver2.getLeftTriggerAxis() > 0.1)
            .whileTrue(new InstantCommand(() -> intake.intake(), intake))
            .onFalse(new InstantCommand(() -> intake.stop(), intake));

        /* Intake Control - D-Pad Up (100%) */
        new Trigger(() -> driver2.getPOV() == 0)
            .whileTrue(new StartEndCommand(
                () -> intake.setPercent(-1.0),
                () -> intake.stop(),
                intake
            ));

        /* Prepare Shot + Auto-Aim - D-Pad Right (hold to aim, spin up shooter, and position hood) */
        new Trigger(() -> driver2.getPOV() == 90)
            .whileTrue(Commands.parallel(
                new AimAndDriveCommand(swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis)),
                new PrepareShotCommand(shooter, hood, swerve::getPose)
            ));

        /* Shooter Control - Right Trigger */
        new Trigger(() -> driver2.getRightTriggerAxis() > 0.1)
            .whileTrue(new ShootCommand(shooter, Flywheel));

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

    public Hood getHood() {
        return hood;
    }
}