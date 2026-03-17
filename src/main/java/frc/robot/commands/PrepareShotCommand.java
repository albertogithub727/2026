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
        // Distance-based RPM and hood position lookup table
        // TODO: Tune these values on the actual field by measuring shots at known distances
        distanceToShotMap.put(Inches.of(36.0),  new Shot(2340, 5));   // Very close - low RPM, hood nearly flat
        distanceToShotMap.put(Inches.of(52.0),  new Shot(3300, 10));    // Close range
        distanceToShotMap.put(Inches.of(78.0),  new Shot(3600, 18)); // Short-mid range
        distanceToShotMap.put(Inches.of(114.4), new Shot(3850, 26)); // Mid range (original)
        distanceToShotMap.put(Inches.of(140.0), new Shot(3900, 30));   // Mid-far range
        distanceToShotMap.put(Inches.of(165.5), new Shot(4000, 33)); // Far range (original)
        distanceToShotMap.put(Inches.of(200.0), new Shot(4100, 35)); // Very far
        distanceToShotMap.put(Inches.of(240.0), new Shot(4150, 40)); // Maximum practical range
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
        return hood.isPositionWithinTolerance() && shooter.isVelocityWithinTolerance();
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
        shooter.setRPM(shot.shooterSpeed);
        hood.setPosition(shot.hoodPosition);

        SmartDashboard.putNumber("Auto Shot/Distance (in)", distanceToHub.in(Inches));
        SmartDashboard.putNumber("Auto Shot/Target RPM", shot.shooterSpeed);
        SmartDashboard.putNumber("Auto Shot/Hood Position", shot.hoodPosition);
        SmartDashboard.putBoolean("Auto Shot/Ready", isReadyToShoot());
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