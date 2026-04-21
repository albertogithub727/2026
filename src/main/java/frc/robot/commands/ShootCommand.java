package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Runnable shooterSpeedSetup;
    private final Timer timer = new Timer();
    private double baseRPM;
    private double rpmBoost;
    private double feederDelay;

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake) {
        this(shooter, flywheel, intake, () -> shooter.setRPM(Constants.Shooter.shooterRPM), Constants.Shooter.shooterRPM, 200, Constants.Shooter.feederDelay);
    }

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake, Runnable shooterSpeedSetup) {
        this(shooter, flywheel, intake, shooterSpeedSetup, 0, 200, Constants.Shooter.feederDelay);
    }

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake, Runnable shooterSpeedSetup, double baseRPM) {
        this(shooter, flywheel, intake, shooterSpeedSetup, baseRPM, 200, Constants.Shooter.feederDelay);
    }

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake, Runnable shooterSpeedSetup, double baseRPM, double rpmBoost) {
        this(shooter, flywheel, intake, shooterSpeedSetup, baseRPM, rpmBoost, Constants.Shooter.feederDelay);
    }

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake, Runnable shooterSpeedSetup, double baseRPM, double rpmBoost, double feederDelay) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.intake = intake;
        this.shooterSpeedSetup = shooterSpeedSetup;
        this.baseRPM = baseRPM;
        this.rpmBoost = rpmBoost;
        this.feederDelay = feederDelay;
        addRequirements(shooter, flywheel, intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        shooterSpeedSetup.run();
    }

    @Override
    public void execute() {
        double elapsed = timer.get();

        if (elapsed >= feederDelay) {
            // RPM boost for the first second after feeder starts (only if using RPM control)
            double timeSinceFeeder = elapsed - feederDelay;
            if (baseRPM > 0) {
                if (timeSinceFeeder < 1.0) {
                    // Boost RPM for first balls (amount configurable per preset)
                    shooter.setRPM(baseRPM + rpmBoost);
                } else {
                    // Return to normal RPM after 1 second
                    shooter.setRPM(baseRPM);
                }
            }

            shooter.runFeeder(Constants.Shooter.feederSpeed);
            flywheel.setVelocity2(-6000);
            intake.intake2();

            // Move intake arm up briefly to help feed balls
            if (elapsed >= 3.0 && elapsed < 4.0) {
                flywheel.setPercent1(-Constants.Shooter.intakeArmOscillateSpeedUp);
            } else {
                flywheel.setPercent1(0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        flywheel.setVelocity2(0);
        flywheel.setPercent1(0);
        intake.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
