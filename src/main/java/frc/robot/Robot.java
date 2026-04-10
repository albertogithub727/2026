package frc.robot;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    /* Shuffleboard Match Info */
    private GenericEntry matchTimeEntry;
    private GenericEntry currentShiftEntry;
    private GenericEntry hubActiveEntry;
    private GenericEntry timeToNextShiftEntry;

    /* Controller Input Tracking */
    private final XboxController driver = new XboxController(0);
    private final XboxController driver2 = new XboxController(1);

    // Driver 1 entries
    private GenericEntry d1_A, d1_B, d1_X, d1_Y;
    private GenericEntry d1_LB, d1_RB, d1_Start, d1_Back;
    private GenericEntry d1_LT, d1_RT;
    private GenericEntry d1_LX, d1_LY, d1_RX, d1_RY;
    private GenericEntry d1_POV;

    // Driver 2 entries
    private GenericEntry d2_A, d2_B, d2_X, d2_Y;
    private GenericEntry d2_LB, d2_RB, d2_Start, d2_Back;
    private GenericEntry d2_LT, d2_RT;
    private GenericEntry d2_LX, d2_LY, d2_RX, d2_RY;
    private GenericEntry d2_POV;

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

        // Shuffleboard match info tab
        ShuffleboardTab matchTab = Shuffleboard.getTab("Match Info");
        matchTimeEntry = matchTab.add("Match Time", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).withSize(2, 1).getEntry();
        currentShiftEntry = matchTab.add("Current Shift", "---")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0).withSize(2, 1).getEntry();
        hubActiveEntry = matchTab.add("Hub Active", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0).withSize(2, 1).getEntry();
        timeToNextShiftEntry = matchTab.add("Next Shift In", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1).withSize(2, 1).getEntry();

        // Shuffleboard controller input tabs
        ShuffleboardTab d1Tab = Shuffleboard.getTab("Driver 1");
        d1_A = d1Tab.add("A", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(1, 1).getEntry();
        d1_B = d1Tab.add("B", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 0).withSize(1, 1).getEntry();
        d1_X = d1Tab.add("X", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).withSize(1, 1).getEntry();
        d1_Y = d1Tab.add("Y", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).withSize(1, 1).getEntry();
        d1_LB = d1Tab.add("LB", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 1).withSize(1, 1).getEntry();
        d1_RB = d1Tab.add("RB", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 1).withSize(1, 1).getEntry();
        d1_Start = d1Tab.add("Start", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 1).withSize(1, 1).getEntry();
        d1_Back = d1Tab.add("Back", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 1).withSize(1, 1).getEntry();
        d1_LT = d1Tab.add("LT", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
        d1_RT = d1Tab.add("RT", 0.0).withPosition(5, 0).withSize(1, 1).getEntry();
        d1_LX = d1Tab.add("Left X", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        d1_LY = d1Tab.add("Left Y", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        d1_RX = d1Tab.add("Right X", 0.0).withPosition(6, 0).withSize(1, 1).getEntry();
        d1_RY = d1Tab.add("Right Y", 0.0).withPosition(6, 1).withSize(1, 1).getEntry();
        d1_POV = d1Tab.add("POV", -1).withPosition(7, 0).withSize(1, 1).getEntry();

        ShuffleboardTab d2Tab = Shuffleboard.getTab("Driver 2");
        d2_A = d2Tab.add("A", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).withSize(1, 1).getEntry();
        d2_B = d2Tab.add("B", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 0).withSize(1, 1).getEntry();
        d2_X = d2Tab.add("X", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).withSize(1, 1).getEntry();
        d2_Y = d2Tab.add("Y", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).withSize(1, 1).getEntry();
        d2_LB = d2Tab.add("LB", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 1).withSize(1, 1).getEntry();
        d2_RB = d2Tab.add("RB", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 1).withSize(1, 1).getEntry();
        d2_Start = d2Tab.add("Start", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 1).withSize(1, 1).getEntry();
        d2_Back = d2Tab.add("Back", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 1).withSize(1, 1).getEntry();
        d2_LT = d2Tab.add("LT", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
        d2_RT = d2Tab.add("RT", 0.0).withPosition(5, 0).withSize(1, 1).getEntry();
        d2_LX = d2Tab.add("Left X", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        d2_LY = d2Tab.add("Left Y", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        d2_RX = d2Tab.add("Right X", 0.0).withPosition(6, 0).withSize(1, 1).getEntry();
        d2_RY = d2Tab.add("Right Y", 0.0).withPosition(6, 1).withSize(1, 1).getEntry();
        d2_POV = d2Tab.add("POV", -1).withPosition(7, 0).withSize(1, 1).getEntry();
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

        // Update Shuffleboard match indicators
        updateMatchInfo();
        updateControllerInputs();
    }

    private void updateMatchInfo() {
        double matchTime = DriverStation.getMatchTime();
        matchTimeEntry.setDouble(Math.max(0, matchTime));

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

        currentShiftEntry.setString(shiftName);
        timeToNextShiftEntry.setDouble(Math.round(timeToNext * 10.0) / 10.0);
        hubActiveEntry.setBoolean(isHubActive());
    }

    private void updateControllerInputs() {
        // Driver 1
        d1_A.setBoolean(driver.getAButton());
        d1_B.setBoolean(driver.getBButton());
        d1_X.setBoolean(driver.getXButton());
        d1_Y.setBoolean(driver.getYButton());
        d1_LB.setBoolean(driver.getLeftBumperButton());
        d1_RB.setBoolean(driver.getRightBumperButton());
        d1_Start.setBoolean(driver.getStartButton());
        d1_Back.setBoolean(driver.getBackButton());
        d1_LT.setDouble(driver.getLeftTriggerAxis());
        d1_RT.setDouble(driver.getRightTriggerAxis());
        d1_LX.setDouble(driver.getLeftX());
        d1_LY.setDouble(driver.getLeftY());
        d1_RX.setDouble(driver.getRightX());
        d1_RY.setDouble(driver.getRightY());
        d1_POV.setInteger(driver.getPOV());

        // Driver 2
        d2_A.setBoolean(driver2.getAButton());
        d2_B.setBoolean(driver2.getBButton());
        d2_X.setBoolean(driver2.getXButton());
        d2_Y.setBoolean(driver2.getYButton());
        d2_LB.setBoolean(driver2.getLeftBumperButton());
        d2_RB.setBoolean(driver2.getRightBumperButton());
        d2_Start.setBoolean(driver2.getStartButton());
        d2_Back.setBoolean(driver2.getBackButton());
        d2_LT.setDouble(driver2.getLeftTriggerAxis());
        d2_RT.setDouble(driver2.getRightTriggerAxis());
        d2_LX.setDouble(driver2.getLeftX());
        d2_LY.setDouble(driver2.getLeftY());
        d2_RX.setDouble(driver2.getRightX());
        d2_RY.setDouble(driver2.getRightY());
        d2_POV.setInteger(driver2.getPOV());
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
        robotContainer.getHood().setPosition(30);

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
        robotContainer.getHood().setPosition(30);

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
