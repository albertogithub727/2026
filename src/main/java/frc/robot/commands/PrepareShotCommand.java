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
        // Distance-based percent output and hood position lookup table
        // Hood positions are now in rotations (motor rotations, not mm)
        // Shooter speed is percent output (0.0 to 1.0)
        // TODO: Tune these values on the actual field by measuring shots at known distances
        distanceToShotMap.put(Inches.of(36.0),  new Shot(0.48, -0));  // Very close - low power, hood nearly flat
        distanceToShotMap.put(Inches.of(52.0),  new Shot(0.53, -1.5));  // Close range
        distanceToShotMap.put(Inches.of(78.0),  new Shot(0.52, -2.1));  // Short-mid range
        distanceToShotMap.put(Inches.of(114.4), new Shot(0.53, -2));  // Mid range (original)
        distanceToShotMap.put(Inches.of(140.0), new Shot(0.58, -2.5));  // Mid-far range
        distanceToShotMap.put(Inches.of(165.5), new Shot(0.61, -2.8));  // Far range (original)
        distanceToShotMap.put(Inches.of(200.0), new Shot(0.56, -2.8));  // Very far
        distanceToShotMap.put(Inches.of(240.0), new Shot(0.80, -3.2));  // Maximum practical range
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
        // In percent output mode, we just check hood position
        // Could add a delay timer here if needed for shooter spin-up
        return hood.isAtTarget();
    }

    private static String getDistanceLabel(double inches) {
        if (inches < 44)   return "Very close (36in)";
        if (inches < 65)   return "Close (52in)";
        if (inches < 96)   return "Short-mid (78in)";
        if (inches < 127)  return "Mid (114in)";
        if (inches < 153)  return "Mid-far (140in)";
        if (inches < 183)  return "Far (166in)";
        if (inches < 220)  return "Very far (200in)";
        return "Max range (240in)";
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

        // Set shooter speed directly based on distance (no ramping)
        shooter.setPercentOutput(shot.shooterSpeed);
        hood.setPosition(shot.hoodPosition);

        double distInches = distanceToHub.in(Inches);
        SmartDashboard.putNumber("Auto Shot/Distance (in)", distInches);
        SmartDashboard.putString("Auto Shot/Range", getDistanceLabel(distInches));
        SmartDashboard.putNumber("Auto Shot/Shooter Percent", shot.shooterSpeed * 100);
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
        hood.setPosition(0);
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