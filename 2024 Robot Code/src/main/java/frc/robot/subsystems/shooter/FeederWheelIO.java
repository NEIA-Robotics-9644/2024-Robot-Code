package frc.robot.subsystems.shooter;

public interface FeederWheelIO {

    public void spinWheel(double normalizedVelocity);

    public double getVelocityPercent();

    public void periodic();

    public void setBrakeMode(boolean brake);
}
