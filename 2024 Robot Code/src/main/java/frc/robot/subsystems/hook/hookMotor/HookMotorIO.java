package frc.robot.subsystems.hook.hookMotor;


public interface HookMotorIO {
    public void spinMotor(double normalizedVelocity);

    public double getMotorRotations();

    public double getMotorVelocityRPM();

    public void periodic();
}
