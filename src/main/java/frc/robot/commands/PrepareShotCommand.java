package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) ->
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterSpeed, endValue.shooterSpeed, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        // Duty cycle values (0.0 to 1.0) for shooter motors at different distances
        distanceToShotMap.put(Inches.of(52.0), new Shot(0.70, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(0.80, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(0.90, 0.48));
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return hood.isPositionWithinTolerance();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setPercentOutput(shot.shooterSpeed);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
    }

    public static class Shot {
        public final double shooterSpeed;
        public final double hoodPosition;

        public Shot(double shooterSpeed, double hoodPosition) {
            this.shooterSpeed = shooterSpeed;
            this.hoodPosition = hoodPosition;
        }
    }
}
