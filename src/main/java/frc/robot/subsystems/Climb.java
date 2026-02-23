package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

    public Climb() {
        climbMotor = new TalonFX(Constants.Climb.motorID);
        configMotor();
    }

    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = Constants.Climb.motorInvert;
        config.MotorOutput.NeutralMode = Constants.Climb.neutralMode;

        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Climb.enableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Climb.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = Constants.Climb.enableCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Constants.Climb.statorCurrentLimit;

        climbMotor.getConfigurator().apply(config);
    }

    public void climbUp() {
        dutyCycleControl.Output = Constants.Climb.climbSpeed;
        climbMotor.setControl(dutyCycleControl);
    }

    public void climbDown() {
        dutyCycleControl.Output = -Constants.Climb.climbSpeed;
        climbMotor.setControl(dutyCycleControl);
    }

    public void stop() {
        dutyCycleControl.Output = 0;
        climbMotor.setControl(dutyCycleControl);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Motor Output", dutyCycleControl.Output);
    }
}
