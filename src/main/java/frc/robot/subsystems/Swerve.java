package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyro;
    private final SwerveModule[] swerveModules;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;
    private boolean agitating = false;

    /* AdvantageScope 3D field publishers */
    private final StructPublisher<Pose3d> robotPose3dPublisher;
    private final StructPublisher<Pose2d> robotPose2dPublisher;
    private final StructArrayPublisher<SwerveModuleState> actualStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> desiredStatesPublisher;

    /* Simulation state */
    private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[4];
    private double simYawDegrees = 0;

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

        /* AdvantageScope 3D field publishers */
        var nt = NetworkTableInstance.getDefault();
        robotPose3dPublisher = nt.getStructTopic("Robot/Pose3d", Pose3d.struct).publish();
        robotPose2dPublisher = nt.getStructTopic("Robot/Pose2d", Pose2d.struct).publish();
        actualStatesPublisher = nt.getStructArrayTopic("Robot/ActualStates", SwerveModuleState.struct).publish();
        desiredStatesPublisher = nt.getStructArrayTopic("Robot/DesiredStates", SwerveModuleState.struct).publish();

        setupPathPlanner();
    }

    public void setupPathPlanner() {  
    try {
        RobotConfig config;
      config = RobotConfig.fromGUISettings();
          // Configure AutoBuilder for holonomic drive
          AutoBuilder.configure(
            this::getPose,
            // Robot pose supplier
            this::resetPose,
            // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds,    
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speedsRobotRelative, moduleFeedForwards) -> {
                SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speedsRobotRelative);
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxSpeed);
                setModuleStates(moduleStates);
            },
            // Method that will drive the robot given ROB2OT RELATIVE ChassisSpeeds. Also
            // optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
                // PPHolonomicController is the built in path following controller for holonomic
                // drive trains
                new PIDConstants(5.0, 0.0, 0.0),
                // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0)
            // Rotation PID constants
            ),
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        // Reference to this subsystem to set requirements
        ); } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
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
                    getGyroYaw())
                : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation)
        );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        /* Add angle oscillation when agitating */
        if (agitating) {
            double offsetDeg = Math.sin(Timer.getFPGATimestamp() * Constants.Shooter.agitateFrequency * 2 * Math.PI)
                               * Constants.Shooter.agitateAmplitude;
            for (int i = 0; i < swerveModuleStates.length; i++) {
                swerveModuleStates[i].angle = swerveModuleStates[i].angle.plus(Rotation2d.fromDegrees(offsetDeg));
            }
        }

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, agitating);
        }

        lastDesiredStates = swerveModuleStates;
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

        lastDesiredStates = desiredStates;
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

     public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
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

    public void setAgitating(boolean agitating) {
        this.agitating = agitating;
    }

    public boolean isAgitating() {
        return agitating;
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
        field.getObject("Vision Pose").setPose(visionPose);
    }

    @Override
    public void simulationPeriodic() {
        final double dt = 0.02; // 20ms loop period

        if (lastDesiredStates[0] == null) return;

        // Compute the chassis speeds from the last commanded module states
        ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(lastDesiredStates);

        // Integrate heading
        simYawDegrees += Math.toDegrees(speeds.omegaRadiansPerSecond) * dt;
        gyro.getSimState().setRawYaw(simYawDegrees);

        // Update each module's simulated sensors
        for (SwerveModule mod : swerveModules) {
            mod.simulationUpdate(lastDesiredStates[mod.moduleNumber], dt);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with odometry
        poseEstimator.update(getGyroYaw(), getModulePositions());

        field.setRobotPose(getPose());

        // Publish structured data for AdvantageScope 3D field
        robotPose2dPublisher.set(getPose());
        robotPose3dPublisher.set(new Pose3d(getPose()));
        actualStatesPublisher.set(getModuleStates());
        if (lastDesiredStates[0] != null) {
            desiredStatesPublisher.set(lastDesiredStates);
        }

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
