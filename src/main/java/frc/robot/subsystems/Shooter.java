package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX motor3;
    private final TalonFX feeder;

    private final DutyCycleOut dutyCycle1 = new DutyCycleOut(0);
    private final DutyCycleOut dutyCycle2 = new DutyCycleOut(0);
    private final DutyCycleOut dutyCycle3 = new DutyCycleOut(0);
    private final DutyCycleOut feederDutyCycle = new DutyCycleOut(0);

    public Shooter() {
        motor1 = new TalonFX(Constants.Shooter.motor1ID);
        motor2 = new TalonFX(Constants.Shooter.motor2ID);
        motor3 = new TalonFX(Constants.Shooter.motor3ID);
        feeder = new TalonFX(Constants.Shooter.feederID);

        configMotor(motor1, Constants.Shooter.motor1Invert, Constants.Shooter.neutralMode);
        configMotor(motor2, Constants.Shooter.motorInvert, Constants.Shooter.neutralMode);
        configMotor(motor3, Constants.Shooter.motorInvert, Constants.Shooter.neutralMode);
        configMotor(feeder, Constants.Shooter.feederInvert, Constants.Shooter.neutralMode);

        motor1.optimizeBusUtilization();
        motor2.optimizeBusUtilization();
        motor3.optimizeBusUtilization();
        feeder.optimizeBusUtilization();
    }

    private void configMotor(TalonFX motor, InvertedValue invert, NeutralModeValue neutralMode) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = invert;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.enableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = Constants.Shooter.enableCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.statorCurrentLimit;

        motor.getConfigurator().apply(config);
    }

    public void runShooterMotors(double speed) {
        motor1.setControl(dutyCycle1.withOutput(speed));
        motor2.setControl(dutyCycle2.withOutput(speed));
        motor3.setControl(dutyCycle3.withOutput(speed));
    }

    public void runFeeder(double speed) {
        feeder.setControl(feederDutyCycle.withOutput(speed));
    }

    public void stopAll() {
        motor1.setControl(dutyCycle1.withOutput(0));
        motor2.setControl(dutyCycle2.withOutput(0));
        motor3.setControl(dutyCycle3.withOutput(0));
        feeder.setControl(feederDutyCycle.withOutput(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Motor1 Output", motor1.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Feeder Output", feeder.getDutyCycle().getValueAsDouble());
    }
}
