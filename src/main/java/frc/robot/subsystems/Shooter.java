package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX leftMotor, middleMotor, rightMotor, fourthMotor;
    private final TalonFX feeder, feeder2;
    private final List<TalonFX> shooterMotors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut feederDutyCycle = new DutyCycleOut(0);

    private double dashboardTargetRPM = 0.0;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft);
        middleMotor = new TalonFX(Ports.kShooterMiddle);
        rightMotor = new TalonFX(Ports.kShooterRight);
        fourthMotor = new TalonFX(Ports.kShooterFourth);
        feeder = new TalonFX(Ports.kShooterFeeder);
        feeder2 = new TalonFX(Ports.kShooterFeeder2);
        shooterMotors = List.of(leftMotor, middleMotor, rightMotor, fourthMotor);

        configureShooterMotor(leftMotor, InvertedValue.Clockwise_Positive);
        configureShooterMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
        configureShooterMotor(rightMotor, InvertedValue.CounterClockwise_Positive);
        configureShooterMotor(fourthMotor, InvertedValue.Clockwise_Positive);
        configureFeederMotor(feeder, Constants.Shooter.feederInvert);
        configureFeederMotor(feeder2, Constants.Shooter.feeder2Invert);

        SmartDashboard.putData(this);
    }

    private void configureShooterMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(3.4)
                    .withKI(0.5)
                    .withKD(0.01)
                    .withKV(12.25 / KrakenX60.kFreeSpeed.in(RotationsPerSecond))
            );

        motor.getConfigurator().apply(config);
    }

    private void configureFeederMotor(TalonFX motor, InvertedValue invert) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invert)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
            );

        motor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
        for (final TalonFX motor : shooterMotors) {
            motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : shooterMotors) {
            motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
        }
    }

    public void runFeeder(double speed) {
        feeder.setControl(feederDutyCycle.withOutput(speed));
        feeder2.setControl(feederDutyCycle.withOutput(speed));
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public void stopAll() {
        stop();
        runFeeder(0);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM));
    }

    /** Returns true if any shooter motor is spinning above 100 RPM */
    public boolean isActive() {
        return shooterMotors.stream().anyMatch(motor ->
            Math.abs(motor.getVelocity().getValue().in(RPM)) > 100);
    }

    public boolean isVelocityWithinTolerance() {
        return shooterMotors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        initSendable(builder, fourthMotor, "Fourth");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
        builder.addDoubleProperty("Feeder Output", () -> feeder.getDutyCycle().getValueAsDouble(), null);
    }
}
