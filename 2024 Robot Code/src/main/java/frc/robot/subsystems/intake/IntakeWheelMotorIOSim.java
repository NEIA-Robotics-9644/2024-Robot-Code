package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;

public class IntakeWheelMotorIOSim implements IntakeWheelMotorIO {

    // Simulate a simple motor system

    private final double maxSpeed = 6000; // RPM

    private double velocity = 0.0;

    private boolean brake = false;

    private boolean newSetpoint = false;

    private final PIDController motorModel = new PIDController(0.1, 0.0, 0.0);

    
    


    public void periodic() {
        

        if (!newSetpoint) {
            if (brake) {
                velocity = 0.0;
            } else {
                velocity *= 0.99;
            }
        }
        
        newSetpoint = false;        
    }


    public void runMotorAtPercentVelocity(double percentVelocity) {
        motorModel.setSetpoint(maxSpeed * percentVelocity);
        velocity += motorModel.calculate(velocity);
        velocity = Math.max(-maxSpeed, Math.min(maxSpeed, velocity));

        newSetpoint = true;
    }

    public void setBrake(boolean brake) {
        this.brake = brake;
    }

    public double getMotorVelocityPercent() {
        return velocity / maxSpeed;
    }
    
}
