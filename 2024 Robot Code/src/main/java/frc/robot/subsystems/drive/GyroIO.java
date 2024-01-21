package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
    /**
     * returns the continuous yaw angle of the gyro     
     */
    public Rotation2d getHeading();

    /**
     * resets the gyro's yaw angle to 0
     */
    public void resetHeading();
}