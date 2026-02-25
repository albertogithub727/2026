package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;

public class AimAndDriveCommand extends Command {
    private static final double kAimToleranceDegrees = 5.0;
    private static final double kRotationKP = 1.0;
    private static final double kRotationKI = 0.0;
    private static final double kRotationKD = 0.0;

    private final Swerve swerve;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier strafeInput;
    private final PIDController rotationPID;

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier strafeInput
    ) {
        this.swerve = swerve;
        this.forwardInput = forwardInput;
        this.strafeInput = strafeInput;
        this.rotationPID = new PIDController(kRotationKP, kRotationKI, kRotationKD);
        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(kAimToleranceDegrees);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        return rotationPID.atSetpoint();
    }

    private Rotation2d getDirectionToHub() {
        Translation2d hubPosition = Landmarks.hubPosition();
        Translation2d robotPosition = swerve.getPose().getTranslation();
        return hubPosition.minus(robotPosition).getAngle();
    }

    @Override
    public void execute() {
        double forward = MathUtil.applyDeadband(forwardInput.getAsDouble(), Constants.stickDeadband);
        double strafe = MathUtil.applyDeadband(strafeInput.getAsDouble(), Constants.stickDeadband);

        Rotation2d targetAngle = getDirectionToHub();
        Rotation2d currentAngle = swerve.getHeading();

        double rotationOutput = rotationPID.calculate(
            currentAngle.getDegrees(),
            targetAngle.getDegrees()
        );

        rotationOutput = MathUtil.clamp(rotationOutput,
            -Constants.Swerve.maxAngularVelocity,
            Constants.Swerve.maxAngularVelocity);

        swerve.drive(
            new Translation2d(forward, strafe).times(Constants.Swerve.maxSpeed),
            rotationOutput,
            true,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
