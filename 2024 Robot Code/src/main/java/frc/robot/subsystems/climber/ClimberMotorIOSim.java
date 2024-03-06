package frc.robot.subsystems.climber;

public class ClimberMotorIOSim implements ClimberMotorIO {

    private double motorRotations = 0;
    private double motorVelocityRPM = 0;
    private double normalizedVelocity = 0;

    @Override
    public void spinMotor(double normalizedVelocity) {
        this.normalizedVelocity = normalizedVelocity;
    }

    @Override
    public double getMotorRotations() {
        return motorRotations;
    }

    @Override
    public double getMotorVelocityRPM() {
        return motorVelocityRPM;
    }

    @Override
    public void periodic() {
        double velocity = normalizedVelocity * 1000;
        motorRotations += velocity / 600;
        motorVelocityRPM = velocity;
    }
    
}
