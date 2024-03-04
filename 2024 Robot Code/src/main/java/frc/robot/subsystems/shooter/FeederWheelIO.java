package frc.robot.subsystems.shooter;

public interface FeederWheelIO {

    public void spinWheel(double normalizedVelocity);

    public double getSpeedPercent();

    public void periodic();

    public void setBrakeMode(boolean brake);
}
