package frc.robot.subsystems.testMotorComponent.testMotor;


public interface TestMotorIO {
    public void spinMotor(double normalizedVelocity);

    public double getMotorRotations();

    public double getMotorVelocityRPM();

    public void periodic();
}
