package frc.robot.subsystems.climber.climberMotor;


public interface ClimberMotorIO {
    public void spinMotor(double normalizedVelocity);

    public double getMotorRotations();

    public double getMotorVelocityRPM();

    public void periodic();

    public void setInverted(boolean inverted);
}
