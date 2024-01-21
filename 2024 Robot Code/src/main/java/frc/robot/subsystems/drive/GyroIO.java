package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
    public Rotation2d getHeading();

    public void resetHeading();
}