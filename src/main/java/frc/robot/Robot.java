package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    /* Controller Input Tracking */
    private final XboxController driver = new XboxController(0);
    private final XboxController driver2 = new XboxController(1);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want to run during disabled, autonomous, teleoperated and test.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.
        CommandScheduler.getInstance().run();

        // Update match indicators and controller inputs
        updateMatchInfo();
        updateControllerInputs();
    }

    private void updateMatchInfo() {
        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Info/Match Time", Math.max(0, matchTime));

        // Shift boundaries (teleop time remaining):
        // >130s = Transition, >105s = Shift 1, >80s = Shift 2,
        // >55s = Shift 3, >30s = Shift 4, <=30s = End Game
        String shiftName;
        double timeToNext;
        if (!DriverStation.isTeleopEnabled()) {
            shiftName = DriverStation.isAutonomousEnabled() ? "Auto" : "---";
            timeToNext = 0;
        } else if (matchTime > 130) {
            shiftName = "Transition";
            timeToNext = matchTime - 130;
        } else if (matchTime > 105) {
            shiftName = "Shift 1";
            timeToNext = matchTime - 105;
        } else if (matchTime > 80) {
            shiftName = "Shift 2";
            timeToNext = matchTime - 80;
        } else if (matchTime > 55) {
            shiftName = "Shift 3";
            timeToNext = matchTime - 55;
        } else if (matchTime > 30) {
            shiftName = "Shift 4";
            timeToNext = matchTime - 30;
        } else {
            shiftName = "End Game";
            timeToNext = matchTime;
        }

        SmartDashboard.putString("Match Info/Current Shift", shiftName);
        SmartDashboard.putNumber("Match Info/Next Shift In", Math.round(timeToNext * 10.0) / 10.0);
        SmartDashboard.putBoolean("Match Info/Hub Active", isHubActive());

        // Blink warning 5 seconds before the next shift (~4 Hz toggle)
        boolean shiftWarning = false;
        if (DriverStation.isTeleopEnabled() && timeToNext > 0 && timeToNext <= 5.0) {
            shiftWarning = (int)(edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 4) % 2 == 0;
        }
        SmartDashboard.putBoolean("Match Info/Shift Warning", shiftWarning);
    }

    private void updateControllerInputs() {
        // Driver 1
        SmartDashboard.putBoolean("Driver 1/A", driver.getAButton());
        SmartDashboard.putBoolean("Driver 1/B", driver.getBButton());
        SmartDashboard.putBoolean("Driver 1/X", driver.getXButton());
        SmartDashboard.putBoolean("Driver 1/Y", driver.getYButton());
        SmartDashboard.putBoolean("Driver 1/LB", driver.getLeftBumperButton());
        SmartDashboard.putBoolean("Driver 1/RB", driver.getRightBumperButton());
        SmartDashboard.putBoolean("Driver 1/Start", driver.getStartButton());
        SmartDashboard.putBoolean("Driver 1/Back", driver.getBackButton());
        SmartDashboard.putNumber("Driver 1/LT", driver.getLeftTriggerAxis());
        SmartDashboard.putNumber("Driver 1/RT", driver.getRightTriggerAxis());
        SmartDashboard.putNumber("Driver 1/Left X", driver.getLeftX());
        SmartDashboard.putNumber("Driver 1/Left Y", driver.getLeftY());
        SmartDashboard.putNumber("Driver 1/Right X", driver.getRightX());
        SmartDashboard.putNumber("Driver 1/Right Y", driver.getRightY());
        SmartDashboard.putNumber("Driver 1/POV", driver.getPOV());

        // Driver 2
        SmartDashboard.putBoolean("Driver 2/A", driver2.getAButton());
        SmartDashboard.putBoolean("Driver 2/B", driver2.getBButton());
        SmartDashboard.putBoolean("Driver 2/X", driver2.getXButton());
        SmartDashboard.putBoolean("Driver 2/Y", driver2.getYButton());
        SmartDashboard.putBoolean("Driver 2/LB", driver2.getLeftBumperButton());
        SmartDashboard.putBoolean("Driver 2/RB", driver2.getRightBumperButton());
        SmartDashboard.putBoolean("Driver 2/Start", driver2.getStartButton());
        SmartDashboard.putBoolean("Driver 2/Back", driver2.getBackButton());
        SmartDashboard.putNumber("Driver 2/LT", driver2.getLeftTriggerAxis());
        SmartDashboard.putNumber("Driver 2/RT", driver2.getRightTriggerAxis());
        SmartDashboard.putNumber("Driver 2/Left X", driver2.getLeftX());
        SmartDashboard.putNumber("Driver 2/Left Y", driver2.getLeftY());
        SmartDashboard.putNumber("Driver 2/Right X", driver2.getRightX());
        SmartDashboard.putNumber("Driver 2/Right Y", driver2.getRightY());
        SmartDashboard.putNumber("Driver 2/POV", driver2.getPOV());
    }

    private boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true; // data not yet received, assume active

        boolean redInactiveFirst = (gameData.charAt(0) == 'R');
        boolean shift1Active = (alliance.get() == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;

        if (matchTime > 130) return true;        // Transition - both active
        else if (matchTime > 105) return shift1Active;   // Shift 1
        else if (matchTime > 80) return !shift1Active;   // Shift 2
        else if (matchTime > 55) return shift1Active;    // Shift 3
        else if (matchTime > 30) return !shift1Active;   // Shift 4
        else return true;                                // End Game - both active
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your RobotContainer class. */
    @Override
    public void autonomousInit() {
        //robotContainer.getHood().setPosition(0);

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {

        // This makes sure that the autonomous stops running when teleop starts running.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
