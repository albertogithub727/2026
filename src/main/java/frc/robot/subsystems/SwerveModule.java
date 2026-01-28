package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    /* Status Signals */
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Angle> anglePosition;
    private final StatusSignal<Angle> cancoderPosition;

    /* Control Requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0);
    private final PositionVoltage anglePositionControl = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        /* Get Status Signals */
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        anglePosition = angleMotor.getPosition();
        cancoderPosition = angleEncoder.getAbsolutePosition();

        /* Set update frequencies */
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            drivePosition,
            driveVelocity,
            anglePosition,
            cancoderPosition
        );

        driveMotor.optimizeBusUtilization();
        angleMotor.optimizeBusUtilization();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocityControl.Velocity = metersPerSecondToRotationsPerSecond(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocityControl);
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less than 1%. Prevents jittering.
        if (Math.abs(desiredState.speedMetersPerSecond) < (Constants.Swerve.maxSpeed * 0.01)) {
            return;
        }

        anglePositionControl.Position = desiredState.angle.getRotations();
        angleMotor.setControl(anglePositionControl);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(anglePosition.refresh().getValueAsDouble());
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(cancoderPosition.refresh().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            rotationsToMeters(drivePosition.refresh().getValueAsDouble()),
            getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            rotationsToMeters(drivePosition.refresh().getValueAsDouble()),
            getAngle()
        );
    }

    private void configAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        config.MagnetSensor.MagnetOffset = -angleOffset.getRotations();
        angleEncoder.getConfigurator().apply(config);
    }

    private void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* Motor Output */
        config.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        config.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio */
        config.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleSupplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Constants.Swerve.angleStatorCurrentLimit;

        /* PID Config */
        config.Slot0.kP = Constants.Swerve.angleKP;
        config.Slot0.kI = Constants.Swerve.angleKI;
        config.Slot0.kD = Constants.Swerve.angleKD;

        /* Enable continuous wrap for position control */
        config.ClosedLoopGeneral.ContinuousWrap = true;

        angleMotor.getConfigurator().apply(config);

        /* Reset to absolute position from CANcoder on boot */
        resetToAbsolute();
    }

    private void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* Motor Output */
        config.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        config.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio */
        config.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveSupplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveStatorCurrentLimit;

        /* PID Config */
        config.Slot0.kP = Constants.Swerve.driveKP;
        config.Slot0.kI = Constants.Swerve.driveKI;
        config.Slot0.kD = Constants.Swerve.driveKD;
        config.Slot0.kS = Constants.Swerve.driveKS;
        config.Slot0.kV = Constants.Swerve.driveKV;
        config.Slot0.kA = Constants.Swerve.driveKA;

        driveMotor.getConfigurator().apply(config);
        driveMotor.setPosition(0);
    }

    public void resetToAbsolute() {
        /* Get position directly from encoder (not status signal) to avoid null during init */
        double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble();
        angleMotor.setPosition(absolutePosition);
    }

    private double rotationsToMeters(double rotations) {
        return rotations * Constants.Swerve.wheelCircumference;
    }

    private double metersPerSecondToRotationsPerSecond(double metersPerSecond) {
        return metersPerSecond / Constants.Swerve.wheelCircumference;
    }
}