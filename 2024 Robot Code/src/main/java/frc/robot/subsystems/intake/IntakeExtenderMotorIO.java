package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeExtenderMotorIO {
    public void setMotorVoltage(double voltage);

    public Rotation2d getAbsoluteRotation();

    public void setBrake(boolean brake);

    public double getMotorVelocity();

    public void periodic();
}
