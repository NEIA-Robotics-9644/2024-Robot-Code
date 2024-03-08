package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;

public class ShooterAngleIOSim implements ShooterAngleIO {
    
    private double angle = 0.0;

    private double bottomLimit = 0.0;
    private double topLimit = 90.0;

    private final PIDController angleFeedback = new PIDController(0.02, 0.0, 0.0);

    private boolean manualControl = false;

    @Override
    public void setAngleDeg(double setpoint) {
        angleFeedback.setSetpoint(setpoint);
    }

    @Override
    public void periodic() {


        if (!manualControl) {
            double delta = angleFeedback.calculate(angle);
            angle = Math.max(bottomLimit, Math.min(topLimit, angle + delta));
        }
    }

    @Override
    public double getAngleDeg() {
        return angle;
    }

    @Override
    public boolean atSetpoint() {
        return angleFeedback.atSetpoint();
    }


    @Override
    public void setManualControl(boolean enabled) {
        manualControl = enabled;
    }

    @Override
    public boolean manualControlEnabled() {
        return manualControl;
    }

    @Override
    public void setManualVelocityDegPerSec(double velocity) {
        if (manualControl) {
            double delta = velocity * 0.02;
            angle = Math.max(bottomLimit, Math.min(topLimit, angle + delta));
        }
    }


    @Override
    public boolean atBottom() {
        return angle <= bottomLimit + 0.1;
    }

    @Override
    public boolean atTop() {
        return angle >= topLimit - 0.1;
    }

    @Override
    public void resetAngleToBottom() {
        angle = bottomLimit;
    }

    @Override
    public double getTopAngleDeg() {
        return topLimit;
    }

    @Override
    public double getBottomAngleDeg() {
        return bottomLimit;
    }

    @Override
    public double getVelocityPercent() {
        System.out.println("Not implemented yet");
        return 0;
    }

    @Override
    public double averageAcceleration() {
        System.out.println("Not implemented yet");
        return 0;
    }
}
