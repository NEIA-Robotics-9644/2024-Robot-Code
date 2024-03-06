package frc.robot.subsystems.climber;


public interface ClimberMotorIO {
    public void spinMotor(double normalizedVelocity);

    public double getMotorRotations();

    public double getMotorVelocityRPM();

    public void periodic();
}
