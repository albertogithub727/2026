package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Swerve swerve;
    private final Flywheel flywheel;
    private final Timer timer = new Timer();

    public ShootCommand(Shooter shooter, Swerve swerve, Flywheel flywheel) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.flywheel = flywheel;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
        shooter.runShooterMotors(Constants.Shooter.shooterSpeed);
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(Constants.Shooter.feederDelay)) {
            shooter.runFeeder(Constants.Shooter.feederSpeed);
            swerve.setAgitating(true);
            flywheel.setVelocity2(-2000);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        swerve.setAgitating(false);
        flywheel.setVelocity2(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
