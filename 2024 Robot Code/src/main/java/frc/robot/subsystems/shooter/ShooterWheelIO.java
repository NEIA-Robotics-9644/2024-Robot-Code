package frc.robot.subsystems.shooter;

public interface ShooterWheelIO {

    public void spinWheel(double normalizedVelocity);

    public double getVelocityPercent();

    public void periodic();

    public void setInverted(boolean inverted);

}
