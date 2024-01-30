package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;


import frc.robot.Constants.CANBusIDs;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(CANBusIDs.kPigeon2CanID);

    private final double kPigeonRotationOffsetDeg = 90.0;
    /**
     * returns the continuous yaw angle of the gyro     
     */
    public double getAngleDeg() {
        return kPigeonRotationOffsetDeg + pigeon.getAngle();
    }

    /**
     * resets the gyro's yaw angle to 0
     */
    public void resetHeading() {
        pigeon.setYaw(0);
    }
}