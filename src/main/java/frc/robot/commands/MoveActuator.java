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
    
    // How much to change position per cycle (0.02 = 2% per 20ms)
    private static final double POSITION_INCREMENT = 0.02;
    
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
        // Increment or decrement position
        if (extending) {
            targetPosition += POSITION_INCREMENT;
            if (targetPosition > 1.0) {
                targetPosition = 1.0;
            }
        } else {
            targetPosition -= POSITION_INCREMENT;
            if (targetPosition < 0.0) {
                targetPosition = 0.0;
            }
        }
        
        // Send new position to actuator
        actuator.setPosition(targetPosition);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Hold current position when button released
        System.out.println("MoveActuator stopped at position: " + targetPosition);
        actuator.setPosition(targetPosition);
    }
    
    @Override
    public boolean isFinished() {
        // Command runs until button is released or limit reached
        if (extending) {
            return targetPosition >= 1.0;
        } else {
            return targetPosition <= 0.0;
        }
    }
}