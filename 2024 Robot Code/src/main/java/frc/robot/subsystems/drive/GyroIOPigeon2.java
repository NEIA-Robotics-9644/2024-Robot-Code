package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.CANBusIDs;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(CANBusIDs.kPigeon2CanID);

    /**
     * returns the continuous yaw angle of the gyro     
     */
    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    /**
     * resets the gyro's yaw angle to 0
     */
    public void resetHeading() {
        pigeon.setYaw(0);
    }
}