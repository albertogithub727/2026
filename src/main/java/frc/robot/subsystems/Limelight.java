package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    // ── Filtering Thresholds ─────────────────────────────────────────────────

    /**
     * Maximum robot angular velocity (deg/s) at which we still accept vision
     * updates. Above this, motion blur and gyro lag make MegaTag2 unreliable.
     * 720 deg/s = 2 full rotations per second — well above normal driving.
     */
    private static final double MAX_ANGULAR_VEL_DEG_S = 720.0;

    /**
     * Maximum distance (m) from robot to a tag before we reject a SINGLE-tag
     * pose. Multiple tags are less sensitive to range. 3.5 m is roughly the
     * far side of the NEUTRAL ZONE from the HUB.
     */
    private static final double MAX_SINGLE_TAG_DIST_M = 3.5;

    /**
     * Maximum pose ambiguity for a single-tag pose (0-1).
     * Above ~0.7 there are two nearly equal candidate poses — reject it.
     */
    private static final double MAX_AMBIGUITY = 0.7;

    // ── Standard Deviation Tuning ────────────────────────────────────────────
    //
    // VecBuilder.fill(xMeters, yMeters, rotationRadians)
    // LARGER = less trust. SMALLER = more trust.
    //
    // Rotation is very high (9999999) because the Pigeon2 gyro is far more
    // accurate for heading than any vision estimate. Letting vision correct
    // rotation causes fighting between the gyro and the camera.
    //
    // TUNING: If the robot pose jumps around during matches -> INCREASE X/Y.
    //         If wheel odometry drifts badly at the end of paths -> DECREASE X/Y.

    /** Used when 2+ tags are visible — highly reliable translation estimate. */
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
        VecBuilder.fill(0.1, 0.1, 9999999);

    /** Used when exactly 1 tag is visible — less reliable. */
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(0.4, 0.4, 9999999);

    // ── Instance Fields ───────────────────────────────────────────────────────

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    /** Latest angular velocity in deg/s. Updated by Swerve each loop. */
    private double latestAngularVelDegS = 0.0;

    private int acceptedCount = 0;
    private int rejectedCount = 0;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    // ── Vision Measurement ────────────────────────────────────────────────────

    /**
     * Call this every loop (from the updateVisionCommand in RobotContainer)
     * with the current robot angular velocity so we can filter bad estimates.
     */
    public void setAngularVelocity(double angularVelDegS) {
        this.latestAngularVelDegS = angularVelDegS;
    }

    /**
     * Returns a validated vision measurement, or Optional.empty() if the data
     * should be rejected.
     *
     * KEY CHANGES FROM ORIGINAL:
     *   1. Dynamic std devs: more tags = more trust (was fixed 0.1/0.1)
     *   2. Angular velocity gate: rejects when robot is spinning fast
     *   3. Single-tag distance check: rejects far single-tag estimates
     *   4. Single-tag ambiguity check: rejects ambiguous single-tag poses
     *   5. Rotation std dev is now 9999999: fully trusts gyro for heading
     *      (the old MegaTag1 rotation blend was fighting the Pigeon2)
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // MegaTag1: no gyro heading needed — solves pose directly from tag geometry
        final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1 == null || mt1.tagCount == 0) {
            return Optional.empty();
        }

        // Reject if robot is spinning too fast — motion blur corrupts the pose
        if (Math.abs(latestAngularVelDegS) > MAX_ANGULAR_VEL_DEG_S) {
            rejectedCount++;
            return Optional.empty();
        }

        // Single-tag specific checks (more important for MT1 since no gyro assist)
        if (mt1.tagCount == 1 && mt1.rawFiducials.length > 0) {
            LimelightHelpers.RawFiducial tag = mt1.rawFiducials[0];

            if (tag.distToCamera > MAX_SINGLE_TAG_DIST_M) {
                rejectedCount++;
                return Optional.empty();
            }

            if (tag.ambiguity > MAX_AMBIGUITY) {
                rejectedCount++;
                return Optional.empty();
            }
        }

        // More tags = tighter (smaller) std devs = more trust in vision
        final Matrix<N3, N1> stdDevs = mt1.tagCount >= 2
            ? MULTI_TAG_STD_DEVS
            : SINGLE_TAG_STD_DEVS;

        posePublisher.set(mt1.pose);
        acceptedCount++;

        return Optional.of(new Measurement(mt1, stdDevs));
    }

    @Override
    public void periodic() {
        boolean hasTarget = LimelightHelpers.getTV(name);
        SmartDashboard.putBoolean(name + "/Has Target", hasTarget);
        SmartDashboard.putNumber(name + "/TX", LimelightHelpers.getTX(name));
        SmartDashboard.putNumber(name + "/TY", LimelightHelpers.getTY(name));
        SmartDashboard.putNumber(name + "/TA", LimelightHelpers.getTA(name));
        SmartDashboard.putNumber(name + "/Angular Vel Filter (deg/s)", latestAngularVelDegS);
        SmartDashboard.putNumber(name + "/Accepted", acceptedCount);
        SmartDashboard.putNumber(name + "/Rejected", rejectedCount);

        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        SmartDashboard.putNumber(name + "/Tag Count", mt1 != null ? mt1.tagCount : 0);
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}
