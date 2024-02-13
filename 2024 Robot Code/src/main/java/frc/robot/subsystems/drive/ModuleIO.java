package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    /**
     * Gets the drive motor's velocity in RPM
     */
    public double getDriveVelocity();

    public double getTurnP();

    public double getTurnI();

    public double getTurnD();

    public void setDriveVoltage(double volts);

    public void setTurnVoltage(double volts);

    public Rotation2d getAbsoluteRotation();

    public void setDriveBrakeMode(boolean enable);

    public void setTurnBrakeMode(boolean enable);

    public void periodic();
}