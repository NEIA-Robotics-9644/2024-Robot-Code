package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteSensorIOSim implements NoteSensorIO {
    
    private boolean noteDetected = false;

    // Simulate a simple sensor
    // Use an input from SmartDashboard to simulate the sensor

    public NoteSensorIOSim() {
        // Set up the SmartDashboard input
        SmartDashboard.putBoolean("Note Detected Sim Input", false);
    }

    public boolean noteDetected() {
        // Read the SmartDashboard input
        noteDetected = SmartDashboard.getBoolean("Note Detected Sim Input", false);
        return noteDetected;
    }

    public void setDisplayLight(boolean on) {
        SmartDashboard.putBoolean("Sim Note Detected", on);
    }
}
