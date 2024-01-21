package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    /**
     * Gets the drive motor's velocity in RPM
     */
    public double getDriveVelocity();

    public void setDriveVoltage(double voltage);

    public void setTurnVoltage(double voltage);

    public Rotation2d getAbsoluteRotation();

    public void setDriveBrake(boolean brake);

    public void periodic();
}