package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public interface MotorIO {
    public void setMotorVoltage(double voltage);

    public double getMaxMotorVoltage();

    public Rotation2d getAbsoluteRotation();

    public void setBrake(boolean brake);

    public double getMotorVelocity();

    public void periodic();
}
