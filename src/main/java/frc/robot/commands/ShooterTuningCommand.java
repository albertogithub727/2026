package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * Interactive tuning mode for building the distance -> (RPM, hood) lookup
 * table used by PrepareShotCommand.
 *
 * WORKFLOW:
 *   1. Park the robot at a known distance from the HUB. Limelight MUST see
 *      an AprilTag so the pose estimator gives a good distance reading.
 *   2. Hold the tuning button (see RobotContainer binding). Shooter spins up
 *      to the last RPM, hood moves to the last hood position.
 *   3. Adjust RPM and hood with the controller until shots consistently
 *      score (aim for 3/3 makes before logging).
 *   4. LB = log the current (distance, RPM, hood) as a sample.
 *   5. Move to a new distance, repeat.
 *   6. START = dump all samples to /home/lvuser/tuning_samples_<timestamp>.csv
 *      (download it via the roboRIO's FTP/SFTP or `scp` after practice).
 *
 * CONTROLS (on whichever XboxController you pass in):
 *   POV UP        RPM +50
 *   POV DOWN      RPM -50
 *   POV RIGHT     RPM +200  (coarse)
 *   POV LEFT      RPM -200  (coarse)
 *   Y             hood +0.01 rot
 *   A             hood -0.01 rot
 *   B             hood +0.05 rot  (coarse)
 *   X             hood -0.05 rot  (coarse)
 *   RT (>0.1)     run feeder to fire a ball
 *   LB            LOG current sample
 *   RB            UNDO last sample
 *   START         SAVE CSV
 *
 * Samples are kept in memory across re-entries of the command, so you can
 * release the button, reposition, and hold it again without losing data.
 * Samples only clear on a robot reboot.
 */
public class ShooterTuningCommand extends Command {
    // Step sizes — tweak to match how sensitive your shooter/hood are.
    private static final double RPM_FINE_STEP = 50;
    private static final double RPM_COARSE_STEP = 200;
    private static final double HOOD_FINE_STEP = 0.01;   // rotations
    private static final double HOOD_COARSE_STEP = 0.05; // rotations

    // Starting values the first time you enter tuning after a reboot.
    private static final double INITIAL_RPM = 3000;
    private static final double INITIAL_HOOD = 0.10;

    // Safety clamps — prevents fat-fingering a motor into next Tuesday.
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 0;

    private final Shooter shooter;
    private final Hood hood;
    private final Swerve swerve;
    private final XboxController ctrl;

    private double currentRPM = INITIAL_RPM;
    private double currentHoodRot = INITIAL_HOOD;
    private boolean initialized = false;

    // Edge-detect state — we track last frame's button state so every press
    // produces exactly one step, no matter how long the button is held.
    private int lastPov = -1;
    private boolean lastY, lastA, lastB, lastX;
    private boolean lastLB, lastRB, lastStart;

    private final List<Sample> samples = new ArrayList<>();

    /**
     * @param shooter           Shooter subsystem (RPM-controlled).
     * @param hood              Hood subsystem (position-controlled in rotations).
     * @param swerve            Swerve, for reading pose -> distance to hub.
     *                          NOT added as a requirement so the driver can
     *                          still reposition during tuning.
     * @param tuningController  Controller whose inputs drive the tuning UI.
     *                          Ideally a dedicated controller (port 2+) so
     *                          normal teleop bindings don't collide.
     */
    public ShooterTuningCommand(Shooter shooter, Hood hood, Swerve swerve,
                                 XboxController tuningController) {
        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.ctrl = tuningController;
        // Only the things we actually command. Swerve is read-only.
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        if (!initialized) {
            currentRPM = INITIAL_RPM;
            currentHoodRot = INITIAL_HOOD;
            initialized = true;
        }
        // Reset edge-detect state so a button held from before entry doesn't
        // count as a fresh press.
        lastPov = ctrl.getPOV();
        lastY = ctrl.getYButton();
        lastA = ctrl.getAButton();
        lastB = ctrl.getBButton();
        lastX = ctrl.getXButton();
        lastLB = ctrl.getLeftBumperButton();
        lastRB = ctrl.getRightBumperButton();
        lastStart = ctrl.getStartButton();

        applyOutputs();
        SmartDashboard.putString("Tuning/Status", "ACTIVE");
        System.out.println("[Tuning] Entered. RPM=" + currentRPM
            + "  hood=" + currentHoodRot + "  samples held=" + samples.size());
    }

    @Override
    public void execute() {
        handleRpmAdjust();
        handleHoodAdjust();
        handleFeeder();
        handleLog();
        handleUndo();
        handleSave();

        applyOutputs();
        publishTelemetry();
    }

    private void handleRpmAdjust() {
        int pov = ctrl.getPOV();
        if (pov != lastPov) {
            switch (pov) {
                case 0:   currentRPM += RPM_FINE_STEP;   break; // UP
                case 180: currentRPM -= RPM_FINE_STEP;   break; // DOWN
                case 90:  currentRPM += RPM_COARSE_STEP; break; // RIGHT
                case 270: currentRPM -= RPM_COARSE_STEP; break; // LEFT
                default: break;
            }
            currentRPM = MathUtil.clamp(currentRPM, MIN_RPM, MAX_RPM);
            lastPov = pov;
        }
    }

    private void handleHoodAdjust() {
        boolean y = ctrl.getYButton();
        boolean a = ctrl.getAButton();
        boolean b = ctrl.getBButton();
        boolean x = ctrl.getXButton();

        if (y && !lastY) currentHoodRot += HOOD_FINE_STEP;
        if (a && !lastA) currentHoodRot -= HOOD_FINE_STEP;
        if (b && !lastB) currentHoodRot += HOOD_COARSE_STEP;
        if (x && !lastX) currentHoodRot -= HOOD_COARSE_STEP;

        currentHoodRot = MathUtil.clamp(currentHoodRot,
            Constants.Hood.minPosition, Constants.Hood.maxPosition);

        lastY = y; lastA = a; lastB = b; lastX = x;
    }

    private void handleFeeder() {
        // Hold RT to feed a ball into the spinning flywheels.
        if (ctrl.getRightTriggerAxis() > 0.1) {
            shooter.runFeeder(Constants.Shooter.feederSpeed);
        } else {
            shooter.runFeeder(0);
        }
    }

    private void handleLog() {
        boolean lb = ctrl.getLeftBumperButton();
        if (lb && !lastLB) {
            double dist = Landmarks.distanceToHubMeters(swerve.getPose().getTranslation());
            samples.add(new Sample(dist, currentRPM, currentHoodRot));
            System.out.printf("[Tuning] LOGGED #%d: dist=%.3f m (%.1f in)  RPM=%.0f  hood=%.3f rot%n",
                samples.size(), dist, Units.metersToInches(dist), currentRPM, currentHoodRot);
        }
        lastLB = lb;
    }

    private void handleUndo() {
        boolean rb = ctrl.getRightBumperButton();
        if (rb && !lastRB && !samples.isEmpty()) {
            Sample removed = samples.remove(samples.size() - 1);
            System.out.printf("[Tuning] UNDID sample: dist=%.3f m  RPM=%.0f  hood=%.3f rot%n",
                removed.distanceMeters, removed.rpm, removed.hoodRotations);
        }
        lastRB = rb;
    }

    private void handleSave() {
        boolean start = ctrl.getStartButton();
        if (start && !lastStart) {
            saveCsv();
        }
        lastStart = start;
    }

    private void applyOutputs() {
        shooter.setRPM(currentRPM);
        hood.setPosition(currentHoodRot);
    }

    private void publishTelemetry() {
        double dist = Landmarks.distanceToHubMeters(swerve.getPose().getTranslation());
        SmartDashboard.putNumber("Tuning/Distance (m)",       dist);
        SmartDashboard.putNumber("Tuning/Distance (in)",      Units.metersToInches(dist));
        SmartDashboard.putNumber("Tuning/Target RPM",         currentRPM);
        SmartDashboard.putNumber("Tuning/Target Hood (rot)",  currentHoodRot);
        SmartDashboard.putNumber("Tuning/Samples Logged",     samples.size());

        // Show the last sample so the driver can see what they just logged.
        if (!samples.isEmpty()) {
            Sample last = samples.get(samples.size() - 1);
            SmartDashboard.putString("Tuning/Last Sample", String.format(
                "#%d  %.2f in  %.0f RPM  %.3f rot",
                samples.size(),
                Units.metersToInches(last.distanceMeters),
                last.rpm,
                last.hoodRotations));
        }
    }

    private void saveCsv() {
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        File file = new File("/home/lvuser/tuning_samples_" + stamp + ".csv");
        try (PrintWriter pw = new PrintWriter(new FileWriter(file))) {
            pw.println("distance_meters,distance_inches,rpm,hood_rotations");
            for (Sample s : samples) {
                pw.printf("%.4f,%.2f,%.0f,%.4f%n",
                    s.distanceMeters,
                    Units.metersToInches(s.distanceMeters),
                    s.rpm,
                    s.hoodRotations);
            }
            System.out.println("[Tuning] Saved " + samples.size() + " samples to " + file.getAbsolutePath());
            SmartDashboard.putString("Tuning/Last Save", file.getName());
        } catch (IOException e) {
            System.err.println("[Tuning] FAILED to write CSV: " + e.getMessage());
            SmartDashboard.putString("Tuning/Last Save", "FAILED: " + e.getMessage());
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        // Don't clear samples — user may want to re-enter and keep going.
        SmartDashboard.putString("Tuning/Status",
            "Exited (" + samples.size() + " samples held)");
        System.out.println("[Tuning] Exited. " + samples.size() + " samples held in memory.");
    }

    @Override
    public boolean isFinished() {
        return false; // runs while the bound button is held
    }

    private static final class Sample {
        final double distanceMeters;
        final double rpm;
        final double hoodRotations;

        Sample(double distanceMeters, double rpm, double hoodRotations) {
            this.distanceMeters = distanceMeters;
            this.rpm = rpm;
            this.hoodRotations = hoodRotations;
        }
    }
}