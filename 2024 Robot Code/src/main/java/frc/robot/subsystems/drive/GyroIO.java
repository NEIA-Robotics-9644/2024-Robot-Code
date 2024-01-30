package frc.robot.subsystems.drive;

public interface GyroIO {
    
    /**
     * returns the continuous yaw angle of the gyro     
     */
    public double getAngleDeg();

    /**
     * resets the gyro's yaw angle to 0
     */
    public void resetHeading();
}