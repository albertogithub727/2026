package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelMotor2;

    /* Status Signals */
    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<AngularVelocity> flywheelVelocity2;
    private final StatusSignal<Angle> flywheelPosition;
    private final StatusSignal<Angle> flywheelPosition2;

    /* Control Requests */
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final VelocityVoltage velocityControl2 = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleControl2 = new DutyCycleOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(1);
    private final PositionVoltage positionControl2 = new PositionVoltage(0).withSlot(1);

    private double targetVelocityRPS = 0.0;
    private double targetVelocity2RPS = 0.0;
    private double targetPosition = 0.0;
    private double targetPosition2 = 0.0;
    private boolean motor1Extended = false;
    private boolean motor2Extended = false;

    public Flywheel() {
        flywheelMotor = new TalonFX(Constants.Flywheel.motorID);
        flywheelMotor2 = new TalonFX(Constants.Flywheel.motor2ID);
        configMotor(flywheelMotor, Constants.Flywheel.motorInvert);
        configMotor(flywheelMotor2, Constants.Flywheel.motor2Invert);

        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelVelocity.setUpdateFrequency(50);
        flywheelVelocity2 = flywheelMotor2.getVelocity();
        flywheelVelocity2.setUpdateFrequency(50);
        flywheelPosition = flywheelMotor.getPosition();
        flywheelPosition.setUpdateFrequency(50);
        flywheelPosition2 = flywheelMotor2.getPosition();
        flywheelPosition2.setUpdateFrequency(50);

        flywheelMotor.optimizeBusUtilization();
        flywheelMotor2.optimizeBusUtilization();

        // Zero motor positions and stop to prevent movement on enable
        flywheelMotor.setPosition(0);
        flywheelMotor2.setPosition(0);
        stop();
    }

    private void configMotor(TalonFX motor, InvertedValue invert) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* Motor Output */
        config.MotorOutput.Inverted = invert;
        config.MotorOutput.NeutralMode = Constants.Flywheel.neutralMode;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Flywheel.enableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Flywheel.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = Constants.Flywheel.enableCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Constants.Flywheel.statorCurrentLimit;

        /* PID Config for Velocity Control (Slot 0) */
        config.Slot0.kP = Constants.Flywheel.kP;
        config.Slot0.kI = Constants.Flywheel.kI;
        config.Slot0.kD = Constants.Flywheel.kD;
        config.Slot0.kS = Constants.Flywheel.kS;
        config.Slot0.kV = Constants.Flywheel.kV;

        /* PID Config for Position Control (Slot 1) */
        config.Slot1.kP = Constants.Flywheel.positionKP;
        config.Slot1.kI = Constants.Flywheel.positionKI;
        config.Slot1.kD = Constants.Flywheel.positionKD;

        motor.getConfigurator().apply(config);
    }

    /* Motor 1 Control Methods */

    public void setVelocity1(double velocityRPS) {
        targetVelocityRPS = velocityRPS;
        velocityControl.Velocity = velocityRPS;
        flywheelMotor.setControl(velocityControl);
    }

    public void setPercent1(double percent) {
        targetVelocityRPS = 0.0;
        dutyCycleControl.Output = percent;
        flywheelMotor.setControl(dutyCycleControl);
    }

    public void stop1() {
        targetVelocityRPS = 0.0;
        flywheelMotor.setControl(dutyCycleControl.withOutput(0));
    }

    public void setPosition1(double rotations) {
        targetPosition = rotations;
        positionControl.Position = rotations;
        flywheelMotor.setControl(positionControl);
    }

    public double getPosition1() {
        return flywheelPosition.refresh().getValueAsDouble();
    }

    public void toggleMotor1Position(double rotations) {
        if (motor1Extended) {
            setPosition1(getPosition1() - rotations);
        } else {
            setPosition1(getPosition1() + rotations);
        }
        motor1Extended = !motor1Extended;
    }

    public boolean isMotor1Extended() {
        return motor1Extended;
    }

    /* Motor 2 Control Methods */

    public void setVelocity2(double velocityRPS) {
        targetVelocity2RPS = velocityRPS;
        velocityControl2.Velocity = velocityRPS;
        flywheelMotor2.setControl(velocityControl2);
    }

    public void setPercent2(double percent) {
        targetVelocity2RPS = 0.0;
        dutyCycleControl2.Output = percent;
        flywheelMotor2.setControl(dutyCycleControl2);
    }

    public void stop2() {
        targetVelocity2RPS = 0.0;
        flywheelMotor2.setControl(dutyCycleControl2.withOutput(0));
    }

    public void setPosition2(double rotations) {
        targetPosition2 = rotations;
        positionControl2.Position = rotations;
        flywheelMotor2.setControl(positionControl2);
    }

    public double getPosition2() {
        return flywheelPosition2.refresh().getValueAsDouble();
    }

    public void toggleMotor2Position(double rotations) {
        if (motor2Extended) {
            setPosition2(getPosition2() - rotations);
        } else {
            setPosition2(getPosition2() + rotations);
        }
        motor2Extended = !motor2Extended;
    }

    public boolean isMotor2Extended() {
        return motor2Extended;
    }

    /* Both Motors Control Methods */

    public void setVelocity(double velocityRPS) {
        setVelocity1(velocityRPS);
        setVelocity2(velocityRPS);
    }

    public void setPercent(double percent) {
        setPercent1(percent);
        setPercent2(percent);
    }

    public void stop() {
        stop1();
        stop2();
    }

    /**
     * Get the current flywheel velocity in rotations per second.
     * @return Current velocity in RPS
     */
    public double getVelocityRPS() {
        return flywheelVelocity.refresh().getValueAsDouble();
    }

    /**
     * Get the target velocity in rotations per second.
     * @return Target velocity in RPS
     */
    public double getTargetVelocityRPS() {
        return targetVelocityRPS;
    }

    /**
     * Check if the flywheel is at the target velocity within a tolerance.
     * @param toleranceRPS Tolerance in rotations per second
     * @return True if at target velocity
     */
    public boolean atTargetVelocity(double toleranceRPS) {
        return Math.abs(getVelocityRPS() - targetVelocityRPS) < toleranceRPS;
    }

    /**
     * Check if the flywheel is at the target velocity using default tolerance.
     * @return True if at target velocity
     */
    public boolean atTargetVelocity() {
        return atTargetVelocity(Constants.Flywheel.velocityTolerance);
    }

    public double getVelocity2RPS() {
        return flywheelVelocity2.refresh().getValueAsDouble();
    }

    public double getTargetVelocity2RPS() {
        return targetVelocity2RPS;
    }

    public boolean atTargetVelocity1(double toleranceRPS) {
        return Math.abs(getVelocityRPS() - targetVelocityRPS) < toleranceRPS;
    }

    public boolean atTargetVelocity1() {
        return atTargetVelocity1(Constants.Flywheel.velocityTolerance);
    }

    public boolean atTargetVelocity2(double toleranceRPS) {
        return Math.abs(getVelocity2RPS() - targetVelocity2RPS) < toleranceRPS;
    }

    public boolean atTargetVelocity2() {
        return atTargetVelocity2(Constants.Flywheel.velocityTolerance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/Velocity1 RPS", getVelocityRPS());
        SmartDashboard.putNumber("Flywheel/Velocity2 RPS", getVelocity2RPS());
        SmartDashboard.putNumber("Flywheel/Target1 RPS", targetVelocityRPS);
        SmartDashboard.putNumber("Flywheel/Target2 RPS", targetVelocity2RPS);
        SmartDashboard.putBoolean("Flywheel/At Target1", atTargetVelocity1());
        SmartDashboard.putBoolean("Flywheel/At Target2", atTargetVelocity2());
    }
}
