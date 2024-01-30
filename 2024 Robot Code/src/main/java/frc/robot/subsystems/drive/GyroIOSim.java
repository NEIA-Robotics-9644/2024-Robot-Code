package frc.robot.subsystems.drive;

public class GyroIOSim implements GyroIO {

    
    private double angleDeg = 0;

    @Override
    public double getAngleDeg() {
        return angleDeg;
    }

    @Override
    public void resetHeading() {
        angleDeg = 0;
    }

}