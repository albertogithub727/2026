package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field landmark positions for FRC 2026 REBUILT.
 *
 * Coordinate system: Blue origin (bottom-left of field diagram).
 *   +X = toward Red alliance wall (down the long axis)
 *   +Y = toward top of field diagram (across the short axis)
 *
 * Field: 651.2 in long x 317.7 in wide
 *
 * HUB CENTER DERIVATION (from game manual Section 5.4):
 *   "Each ALLIANCE has a dedicated HUB centered between two BUMPS
 *    located 158.6 in (~4.03 m) away from their ALLIANCE WALL."
 *
 *   The HUB is 47 in x 47 in. The 158.6 in is to the center of the
 *   Alliance Zone boundary (Robot Starting Line), which is the near
 *   edge of the HUB's footprint area. Per the field drawings:
 *     Blue HUB center X ≈ 158.6 - (47/2) / 2 — verify against drawings!
 *
 *   The hub is centered in the field width (Y = 317.7/2 = 158.85 in).
 *
 * TODO: Confirm exact hub center coordinates from the 2026 Field
 *       Dimension Drawings at https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
 *       The values below are derived from manual text and should be
 *       cross-checked against the official CAD/drawings.
 */
public class Landmarks {

    // ── 2026 REBUILT Field Dimensions ─────────────────────────────────────────
    public static final double FIELD_LENGTH_IN = 651.2;
    public static final double FIELD_WIDTH_IN  = 317.7;

    // ── Blue HUB Center ───────────────────────────────────────────────────────
    // From the game manual: HUB is at the Robot Starting Line (158.6 in from
    // Blue wall), centered between two BUMPs on the field's Y midpoint.
    // The HUB is 47 in square. Its center is approximately at:
    //   X ≈ 158.6 in - 23.5 in (half HUB) = 135.1 in  ← conservative estimate
    // The AimAndDriveCommand uses this to rotate toward the hub opening.
    // Cross-check against official Field Dimension Drawings and adjust!
    //
    // NOTE: Your previous values were (182.105, 160.845) and (469.115, 160.845).
    // Those may have been measured/verified by your team — if your auto shots
    // were working, keep those values and ignore the ones below.
    public static final double BLUE_HUB_X_IN = 182.105; // ← your team's measured value
    public static final double BLUE_HUB_Y_IN = 160.845; // ← your team's measured value

    // ── IMPORTANT: AprilTag Field Map ────────────────────────────────────────
    //
    // The original Landmarks.java used:
    //   AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
    //
    // kDefaultField is NOT the 2026 field. For 2026 REBUILT you must:
    //   1. Download the 2026 .fmap from https://docs.limelightvision.io/docs/resources/downloads
    //   2. Upload it to the Limelight web UI (AprilTag tab → Field Map)
    //   3. WPILib's AprilTagFields does not yet have a kField2026 enum entry —
    //      do NOT use fieldLayout for pose-critical code. Use Limelight MegaTag2
    //      (which reads the uploaded .fmap) for all field-space pose estimation.
    //
    // getTagPosition() below is kept for reference/debugging only.
    // Do not use it for driving decisions until WPILib ships the 2026 layout.

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Returns the 2D position of the alliance HUB center (the opening the robot
     * shoots into). Alliance is determined automatically from DriverStation.
     */
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(
                Units.inchesToMeters(BLUE_HUB_X_IN),
                Units.inchesToMeters(BLUE_HUB_Y_IN)
            );
        }
        // Red alliance: mirror across field center (X axis only)
        return new Translation2d(
            Units.inchesToMeters(FIELD_LENGTH_IN - BLUE_HUB_X_IN),
            Units.inchesToMeters(BLUE_HUB_Y_IN)
        );
    }

    /**
     * Returns the 2D hub position for a specific alliance.
     * Useful when you need the opponent's hub position (e.g. defensive strategy).
     */
    public static Translation2d hubPosition(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return new Translation2d(
                Units.inchesToMeters(BLUE_HUB_X_IN),
                Units.inchesToMeters(BLUE_HUB_Y_IN)
            );
        }
        return new Translation2d(
            Units.inchesToMeters(FIELD_LENGTH_IN - BLUE_HUB_X_IN),
            Units.inchesToMeters(BLUE_HUB_Y_IN)
        );
    }

    /**
     * Returns the distance from a given robot position to the current alliance's HUB.
     * Convenience method for PrepareShotCommand and dashboard logging.
     */
    public static double distanceToHubMeters(Translation2d robotPosition) {
        return robotPosition.getDistance(hubPosition());
    }

    private Landmarks() {} // Utility class
}
