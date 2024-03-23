package frc.robot.subsystems.shooter;

public class ShooterWheelIOSim implements ShooterWheelIO {

    private double velocityRPM = 0.0;

    private double maxSpeedRPM = 5000.0;

    public double inputVelocity = 0.0;

    private boolean newInput = false;

    public void spinWheel(double velocityPercent) {
        inputVelocity = maxSpeedRPM * velocityPercent;
        newInput = true;
    }

    public double getVelocityPercent() {
        return velocityRPM / maxSpeedRPM;
    }

    public void periodic() {
        
        if (newInput) {
            velocityRPM = inputVelocity;
            
            newInput = false;
        } else {
            velocityRPM = 0.0;
            
        }



    }

    public void setInverted(boolean inverted) {
        // TODO Auto-generated method stub

    }
}
