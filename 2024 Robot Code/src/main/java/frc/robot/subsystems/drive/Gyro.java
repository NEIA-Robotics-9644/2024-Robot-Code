package frc.robot.subsystems.drive;


public class Gyro {
    private final GyroIO io;

    public Gyro() {
        throw new IllegalArgumentException("You must pass in valid hardware");
    }

    public Gyro(GyroIO io) {
        if (io == null) {
            throw new IllegalArgumentException("You must pass in valid hardware");
        }

        this.io = io;
    }

    /**
     * returns the continuous yaw angle of the gyro     
     */
    public double getAngleDeg() {
        return io.getAngleDeg();
    }

    /**
     * resets the gyro's yaw angle to 0
     */
    public void resetHeading() {
        io.resetHeading();
    }
}
