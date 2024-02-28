package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

public interface MotorIO {
    public void setMotorVoltage(double voltage);

    public Rotation2d getAbsoluteRotation();

    public void setBrake(boolean brake);

    public double getMotorVelocity();

    public void periodic();
}
