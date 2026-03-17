package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(160.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(160.845));
    }

    public static Optional<Translation2d> getTagPosition(int tagId) {
        return fieldLayout.getTagPose(tagId)
            .map(pose -> pose.toPose2d().getTranslation());
    }
}
