package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Hood extends SubsystemBase {
    // Effective length accounts for custom servo PWM bounds (1000-2500μs)
    // Calibrated: commanding 41mm produced 66mm actual → scale factor 66/41 ≈ 1.61
    private static final double kServoLengthMm = 161.0;
    private static final double kMaxServoSpeedMmPerSec = 20.0;
    private static final double kMinPositionMm = 1.0;
    private static final double kMaxPositionMm = 80.0;
    private static final double kPositionToleranceMm = 1.0;

    private final Servo leftServo;
    private final Servo rightServo;

    private double currentPositionMm = kMinPositionMm;
    private double targetPositionMm = kMinPositionMm;
    private double lastUpdateTimeSec = 0;

    public Hood() {
        leftServo = new Servo(Ports.kHoodLeftServo);
        rightServo = new Servo(Ports.kHoodRightServo);
        leftServo.setBoundsMicroseconds(2500, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2500, 1800, 1500, 1200, 1000);
        setPosition(currentPositionMm);
        SmartDashboard.putData(this);
    }

    /** Sets hood position in millimeters */
    public void setPosition(double mm) {
        final double clampedMm = MathUtil.clamp(mm, kMinPositionMm, kMaxPositionMm);
        final double servoFraction = clampedMm / kServoLengthMm;
        final double servoFractionRight = MathUtil.clamp(clampedMm + 0.1, kMinPositionMm, kMaxPositionMm) / kServoLengthMm;
        leftServo.set(servoFraction);
        rightServo.set(servoFractionRight);
        targetPositionMm = clampedMm;
    }

    /** Returns current hood position in millimeters */
    public double getPosition() {
        return currentPositionMm;
    }

    public double getMinPosition() {
        return kMinPositionMm;
    }

    public double getMaxPosition() {
        return kMaxPositionMm;
    }

    /** Sets hood to the given position in mm and waits until it arrives */
    public Command positionCommand(double mm) {
        return runOnce(() -> setPosition(mm))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPositionMm, currentPositionMm, kPositionToleranceMm);
    }

    private void updateCurrentPosition() {
        final double currentTimeSec = Timer.getFPGATimestamp();
        final double elapsedSec = currentTimeSec - lastUpdateTimeSec;
        lastUpdateTimeSec = currentTimeSec;

        if (isPositionWithinTolerance()) {
            currentPositionMm = targetPositionMm;
            return;
        }

        final double maxDistanceMm = kMaxServoSpeedMmPerSec * elapsedSec;
        currentPositionMm = targetPositionMm > currentPositionMm
            ? Math.min(targetPositionMm, currentPositionMm + maxDistanceMm)
            : Math.max(targetPositionMm, currentPositionMm - maxDistanceMm);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
        SmartDashboard.putNumber("Hood Position (mm)", currentPositionMm);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position (mm)", () -> currentPositionMm, null);
        builder.addDoubleProperty("Target Position (mm)", () -> targetPositionMm, value -> setPosition(value));
    }
}
