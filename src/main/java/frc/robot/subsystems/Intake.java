package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

    // TODO: Set your intake motor CAN ID
    private static final int INTAKE_MOTOR_ID = 29;
    
    // TODO: Adjust motor invert if needed
    private static final InvertedValue MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    
    // Current limiting
    private static final int SUPPLY_CURRENT_LIMIT = 30;
    private static final int STATOR_CURRENT_LIMIT = 60;
    private static final boolean ENABLE_CURRENT_LIMIT = true;

    public Intake() {
        intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
        configMotor();
    }

    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* Motor Output */
        config.MotorOutput.Inverted = MOTOR_INVERT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

        intakeMotor.getConfigurator().apply(config);
    }

    /**
     * Set the intake motor to a specific percent output
     * @param percent Percent output from -1.0 to 1.0
     */
    public void setPercent(double percent) {
        dutyCycleControl.Output = percent;
        intakeMotor.setControl(dutyCycleControl);
    }

    /**
     * Run the intake at full speed forward
     */
    public void intake() {
        setPercent(1.0);
    }

    /**
     * Run the intake at full speed backward (outtake)
     */
    public void outtake() {
        setPercent(-1.0);
    }

    /**
     * Stop the intake motor
     */
    public void stop() {
        setPercent(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Motor Output", dutyCycleControl.Output);
    }
}