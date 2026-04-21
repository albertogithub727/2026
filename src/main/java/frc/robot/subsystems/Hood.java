package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private final TalonFX motor;
    private final MotionMagicVoltage motionMagicRequest;

    private double targetPositionRotations = Constants.Hood.minPosition;

    public Hood() {
        motor = new TalonFX(Constants.Hood.motorID);
        motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);

        configureMotor();

        // Initialize target to current position so hood doesn't move on startup
        targetPositionRotations = motor.getPosition().getValueAsDouble();

        SmartDashboard.putData(this);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.Inverted = Constants.Hood.motorInvert;
        config.MotorOutput.NeutralMode = Constants.Hood.neutralMode;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Hood.enableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = Constants.Hood.enableCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Constants.Hood.statorCurrentLimit;
        
        // PID configuration (Slot 0)
        config.Slot0.kP = Constants.Hood.kP;
        config.Slot0.kI = Constants.Hood.kI;
        config.Slot0.kD = Constants.Hood.kD;
        config.Slot0.kS = Constants.Hood.kS;
        config.Slot0.kV = Constants.Hood.kV;
        config.Slot0.kG = Constants.Hood.kG;

        // Motion Magic configuration
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.motionMagicCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = Constants.Hood.motionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = Constants.Hood.motionMagicJerk;

        // Software limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Hood.maxPosition;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Hood.minPosition;

        motor.getConfigurator().apply(config);
    }

    /** Sets hood position in rotations */
    public void setPosition(double rotations) {
        targetPositionRotations = MathUtil.clamp(rotations, Constants.Hood.minPosition, Constants.Hood.maxPosition);
        motor.setControl(motionMagicRequest.withPosition(targetPositionRotations));
    }

    /** Manually control hood motor with percent output (-1.0 to 1.0) */
    public void setPercent(double percent) {
        motor.set(percent);
    }

    /** Returns current hood position in rotations */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /** Returns target hood position in rotations */
    public double getTargetPosition() {
        return targetPositionRotations;
    }

    public double getMinPosition() {
        return Constants.Hood.minPosition;
    }

    /** Zeros the internal encoder position without moving the motor */
public void zeroEncoder() {
    motor.setPosition(0);
    targetPositionRotations = 0;
}

    public double getMaxPosition() {
        return Constants.Hood.maxPosition;
    }

    /** Sets hood to down position */
    public void setDown() {
        setPosition(Constants.Hood.downPosition);
    }

    /** Sets hood to up position */
    public void setUp() {
        setPosition(Constants.Hood.upPosition);
    }

    /** Sets hood to the given position in rotations and waits until it arrives */
    public Command positionCommand(double rotations) {
        return runOnce(() -> setPosition(rotations))
            .andThen(Commands.waitUntil(this::isAtTarget));
    }

    /** Command to set hood to down position */
    public Command downCommand() {
        return positionCommand(Constants.Hood.downPosition);
    }

    /** Command to set hood to up position */
    public Command upCommand() {
        return positionCommand(Constants.Hood.upPosition);
    }

    public boolean isAtTarget() {
        return MathUtil.isNear(targetPositionRotations, getPosition(), Constants.Hood.positionTolerance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position (rot)", getPosition());
        SmartDashboard.putNumber("Hood Target (rot)", targetPositionRotations);
        SmartDashboard.putBoolean("Hood At Target", isAtTarget());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position (rot)", this::getPosition, null);
        builder.addDoubleProperty("Target Position (rot)", this::getTargetPosition, this::setPosition);
        builder.addBooleanProperty("At Target", this::isAtTarget, null);
    }
}
