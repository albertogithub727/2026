package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton feederButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton feederButton2 = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton toggleMotor2Button = new JoystickButton(driver, XboxController.Button.kX.value);
    
    /* Left Trigger for Intake - defined in configureButtonBindings() */

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Flywheel Flywheel = new Flywheel();
    private final Intake intake = new Intake();
    

    /* ✅ SendableChooser for Autonomous Selection */
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

        FollowPathCommand.warmupCommand().schedule();
        chooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", chooser);
    }

    

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        
        /* Flywheel Controls */
        feederButton.onTrue(new InstantCommand(() -> Flywheel.setVelocity2(2000)));
        feederButton.onFalse(new InstantCommand(() -> Flywheel.setVelocity2(0)));
        feederButton2.onTrue(new InstantCommand(() -> Flywheel.setVelocity2(-2000)));
        feederButton2.onFalse(new InstantCommand(() -> Flywheel.setVelocity2(0)));

        // Toggle motor 1 position: 5 rotations forward, then 5 rotations back
        toggleMotor2Button.onTrue(new InstantCommand(() -> Flywheel.toggleMotor1Position(-7.25)));

        /* Intake Control - Left Trigger */
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.1)
            .whileTrue(new InstantCommand(() -> intake.intake(), intake))
            .onFalse(new InstantCommand(() -> intake.stop(), intake));
    }

    /**
     * Use this to pass the autonomous command to the main Robot class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
    
    /**
     * Get the swerve subsystem for use elsewhere.
     * @return The Swerve subsystem
     */
    public Swerve getSwerve() {
        return swerve;
    }

    /**
     * Get the intake subsystem for use elsewhere.
     * @return The Intake subsystem
     */
    public Intake getIntake() {
        return intake;
    }
}
