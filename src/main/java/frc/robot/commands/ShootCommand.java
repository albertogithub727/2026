package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Flywheel flywheel;
    private final Runnable shooterSpeedSetup;
    private final Timer timer = new Timer();
    private double lastBopTime = 0.0;
    private boolean bopDirectionPositive = true;
    private double startingPosition = 0.0;

    public ShootCommand(Shooter shooter, Flywheel flywheel) {
        this(shooter, flywheel, () -> shooter.setRPM(Constants.Shooter.shooterRPM));
    }

    public ShootCommand(Shooter shooter, Flywheel flywheel, Runnable shooterSpeedSetup) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.shooterSpeedSetup = shooterSpeedSetup;
        addRequirements(shooter, flywheel);
    }

    @Override
    public void initialize() {
        timer.restart();
        lastBopTime = 0.0;
        bopDirectionPositive = true;
        startingPosition = flywheel.getPosition1();
        shooterSpeedSetup.run();
    }

    @Override
    public void execute() {
        double elapsed = timer.get();

        if (elapsed >= Constants.Shooter.feederDelay) {
            shooter.runFeeder(Constants.Shooter.feederSpeed);

            flywheel.setVelocity2(-2000);
        }

        // Bop intake up and down after delay to dislodge stuck balls
        if (elapsed >= Constants.Shooter.intakeBopDelay) {
            if (elapsed - lastBopTime >= Constants.Shooter.intakeBopInterval) {
                double speed = bopDirectionPositive ? Constants.Shooter.intakeBopSpeed : -Constants.Shooter.intakeBopSpeed;
                flywheel.setPercent1(speed);
                bopDirectionPositive = !bopDirectionPositive;
                lastBopTime = elapsed;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();

        flywheel.setVelocity2(0);
        // Return intake to the exact position it was at when shooting started
        flywheel.setPosition1(startingPosition);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
