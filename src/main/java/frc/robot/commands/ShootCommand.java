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

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake) {
        this(shooter, flywheel, intake, () -> shooter.setRPM(Constants.Shooter.shooterRPM));
    }

    public ShootCommand(Shooter shooter, Flywheel flywheel, Intake intake, Runnable shooterSpeedSetup) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.intake = intake;
        this.shooterSpeedSetup = shooterSpeedSetup;
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

        if (elapsed >= Constants.Shooter.feederDelay) {
            shooter.runFeeder(Constants.Shooter.feederSpeed);
            flywheel.setVelocity2(-500);
            intake.intake2();

            // Oscillate intake arm up and down (like D-pad up/down)
            double timeSinceFeeder = elapsed - Constants.Shooter.feederDelay;
            int cycleIndex = (int) (timeSinceFeeder / Constants.Shooter.intakeArmOscillateInterval);
            if (cycleIndex % 2 == 0) {
                flywheel.setPercent1(-Constants.Shooter.intakeArmOscillateSpeedUp);
            } else {
                flywheel.setPercent1(Constants.Shooter.intakeArmOscillateSpeedDown);
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
