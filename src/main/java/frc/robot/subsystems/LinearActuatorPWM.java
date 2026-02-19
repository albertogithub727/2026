package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Linear actuator subsystem using direct PWM control.
 * This gives you full control over the PWM signal.
 * Use this if the standard Servo class doesn't work.
 */
public class LinearActuatorPWM extends SubsystemBase {
    private final PWM actuator;
    private boolean isExtended = false;
    
    // TODO: Set your actuato   r PWM port (0-9 on roboRIO)
    private static final int ACTUATOR_PWM_PORT = 4;
    
    // Position constants
    private static final double RETRACTED_POSITION = 0.0;
    private static final double EXTENDED_POSITION = 1.5;
    private static final double MIDDLE_POSITION = 0.75;
    
    // PWM timing in microseconds
    // Standard servo range is 1000-2000us
    // Adjust these if needed:
    private static final int MIN_PULSE_US = 1000;   // Standard servo minimum
private static final int MAX_PULSE_US = 2500;
    
    // PWM period (standard is 20ms = 20000us)
    private static final int PWM_PERIOD_US = 15000;
    
    public LinearActuatorPWM() {
        actuator = new PWM(ACTUATOR_PWM_PORT);
        
        // Configure PWM period (20ms is standard for servos)
        actuator.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
        
        System.out.println("========================================");
        System.out.println("LINEAR ACTUATOR (PWM MODE) INITIALIZED");
        System.out.println("PWM Port: " + ACTUATOR_PWM_PORT);
        System.out.println("Pulse Range: " + MIN_PULSE_US + "us to " + MAX_PULSE_US + "us");
        System.out.println("Period: " + PWM_PERIOD_US + "us (20ms)");
        System.out.println("========================================");
        
        // Start retracted
        setRetracted();
    }
    
    /**
     * Set the actuator to a specific position.
     * @param position Position from 0.0 (retracted) to 1.0 (extended)
     */
    public void setPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        
        // Calculate pulse width in microseconds
        int pulseWidthUs = (int)(MIN_PULSE_US + (position * (MAX_PULSE_US - MIN_PULSE_US)));
        
        System.out.println("LinearActuator: Setting position to " + position + 
                         " (" + (position * 140.0) + "mm) - Pulse: " + pulseWidthUs + "us");
        
        // Set the PWM pulse width
        actuator.setPulseTimeMicroseconds(pulseWidthUs);
    }
    
    /**
     * Extend the actuator fully.
     */
    public void setExtended() {
        System.out.println("LinearActuator: EXTENDING to 140mm");
        setPosition(EXTENDED_POSITION);
        isExtended = true;
    }
    
    /**
     * Retract the actuator fully.
     */
    public void setRetracted() {
        System.out.println("LinearActuator: RETRACTING to 0mm");
        setPosition(RETRACTED_POSITION);
        isExtended = false;
    }
    
    /**
     * Move to middle position.
     */
    public void setMiddle() {
        System.out.println("LinearActuator: MIDDLE (70mm)");
        setPosition(MIDDLE_POSITION);
    }
    
    /**
     * Toggle between extended and retracted.
     */
    public void toggle() {
        if (isExtended) {
            setRetracted();
        } else {
            setExtended();
        }
    }
    
    /**
     * Get current commanded pulse width in microseconds.
     * @return Pulse width in microseconds
     */
    public int getPulseWidthUs() {
        return actuator.getPulseTimeMicroseconds();
    }
    
    /**
     * Get commanded position as 0.0 to 1.0.
     * @return Position from 0.0 to 1.0
     */
    public double getPosition() {
        int pulseWidth = getPulseWidthUs();
        return (double)(pulseWidth - MIN_PULSE_US) / (MAX_PULSE_US - MIN_PULSE_US);
    }
    
    /**
     * Get commanded extension in millimeters.
     * @return Extension in mm (0-140)
     */
    public double getExtensionMM() {
        return getPosition() * 140.0;
    }
    
    /**
     * Check if commanded to extended position.
     * @return True if commanded extended
     */
    public boolean isExtended() {
        return isExtended;
    }
    
    /**
     * Disable the PWM output (stops sending pulses).
     * Call this to disable the actuator.
     */
    public void disable() {
        actuator.setDisabled();
        System.out.println("LinearActuator: PWM DISABLED");
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LinearActuator/Position", getPosition());
        SmartDashboard.putNumber("LinearActuator/Extension (mm)", getExtensionMM());
        SmartDashboard.putNumber("LinearActuator/Pulse Width (us)", getPulseWidthUs());
        SmartDashboard.putBoolean("LinearActuator/IsExtended", isExtended);
    }
}