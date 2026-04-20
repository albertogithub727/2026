package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

/**
 * Command to continuously move the actuator while held.
 * Gradually increases or decreases position for smooth control.
 */
public class MoveActuator extends Command {
    private final Hood actuator;
    private final boolean extending; // true = extend, false = retract
    
    // How much to change position per cycle (0.01 rotations per 20ms)
    private static final double POSITION_INCREMENT_ROTATIONS = 0.01;
    
    private double targetPosition;
    
    /**
     * Creates a new MoveActuator command.
     * @param actuator The actuator subsystem
     * @param extending True to extend, false to retract
     */
    public MoveActuator(Hood actuator, boolean extending) {
        this.actuator = actuator;
        this.extending = extending;
        addRequirements(actuator);
    }
    
    @Override
    public void initialize() {
        // Start from current position
        targetPosition = actuator.getPosition();
        System.out.println("MoveActuator started: " + (extending ? "EXTENDING" : "RETRACTING"));
    }
    
    @Override
    public void execute() {
        if (extending) {
            targetPosition += POSITION_INCREMENT_ROTATIONS;
            if (targetPosition > actuator.getMaxPosition()) {
                targetPosition = actuator.getMaxPosition();
            }
        } else {
            targetPosition -= POSITION_INCREMENT_ROTATIONS;
            if (targetPosition < actuator.getMinPosition()) {
                targetPosition = actuator.getMinPosition();
            }
        }

        actuator.setPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MoveActuator stopped at position: " + targetPosition + " rotations");
        actuator.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        if (extending) {
            return targetPosition >= actuator.getMaxPosition();
        } else {
            return targetPosition <= actuator.getMinPosition();
        }
    }
}