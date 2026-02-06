package frc.robot;

import java.lang.annotation.Target;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0; // TODO: Set your Pigeon 2 CAN ID

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(27.0);
        public static final double wheelBase = Units.inchesToMeters(25.5);
        public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),   // Front Left
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),  // Front Right
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),  // Back Left
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)  // Back Right
        );

        /* Module Gear Ratios - MK4i L1 */
        public static final double driveGearRatio = 8.14 / 1.0;
        public static final double angleGearRatio = 150.0 / 7.0; // 21.43:1

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleSupplyCurrentLimit = 25;
        public static final int angleStatorCurrentLimit = 40;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveSupplyCurrentLimit = 35;
        public static final int driveStatorCurrentLimit = 60;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 3.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKS = 0.0;
        public static final double driveKV = 0.0;
        public static final double driveKA = 0.0;

        /* Swerve Profiling Values */
        /* Kraken X60 with MK4i L1 - theoretical max ~4.5 m/s */
        public static final double maxSpeed = 6; // meters per second
        public static final double maxAngularVelocity = 10.0; // radians per second
 
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 34;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(222);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(19);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Flywheel {
        public static final int motorID = 9; // TODO: Set your flywheel motor CAN ID
        public static final int motor2ID = 10; // TODO: Set your second flywheel motor CAN ID

        /* Motor Invert */
        public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue motor2Invert = InvertedValue.CounterClockwise_Positive; // TODO: Adjust if needed

        /* Neutral Mode */
        public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;

        /* Current Limiting */
        public static final int supplyCurrentLimit = 40;
        public static final int statorCurrentLimit = 80;
        public static final boolean enableCurrentLimit = true;

        /* PID Values for Velocity Control (Slot 0) */
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0; // Static friction compensation
        public static final double kV = 0.12; // Velocity feedforward

        /* PID Values for Position Control (Slot 1) */
        public static final double positionKP = 1.0; // TODO: Tune this value
        public static final double positionKI = 0.0;
        public static final double positionKD = 0.0;

        /* Velocity Tolerance for atTargetVelocity check (rotations per second) */
        public static final double velocityTolerance = 2.0;

        /* Position Tolerance (rotations) */
        public static final double positionTolerance = 0.1;
    }
}
