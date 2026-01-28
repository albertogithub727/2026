package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyro;
    private final SwerveModule[] swerveModules;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    /**
     * Primary method for controlling the drivetrain.
     * @param translation The desired translation vector (x, y) in m/s
     * @param rotation The desired rotation rate in rad/s
     * @param fieldRelative Whether the translation is field-relative
     * @param isOpenLoop Whether to use open-loop (duty cycle) or closed-loop (velocity) control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getHeading())
                : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation)
        );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Set the desired states for all modules.
     * @param desiredStates Array of desired module states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Get the current robot-relative chassis speeds.
     * @return ChassisSpeeds representing current movement
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Drive the robot using ChassisSpeeds.
     * @param chassisSpeeds The desired chassis speeds
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    /**
     * Get the current states of all swerve modules.
     * @return Array of SwerveModuleState
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Get the current positions of all swerve modules.
     * @return Array of SwerveModulePosition
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Get the estimated robot pose from odometry.
     * @return Pose2d of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the pose estimator to a specific pose.
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * Get the robot's heading (yaw) from the pose estimator.
     * @return Rotation2d representing the robot's heading
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the raw gyro yaw.
     * @return Rotation2d from the Pigeon2
     */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /**
     * Zero the gyro.
     */
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /**
     * Set the gyro to a specific angle.
     * @param angle The angle to set in degrees
     */
    public void setGyro(double angle) {
        gyro.setYaw(angle);
    }

    /**
     * Reset all modules to their absolute encoder positions.
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator
        poseEstimator.update(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());

        // Log data to SmartDashboard
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Theta", getPose().getRotation().getDegrees());

        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
