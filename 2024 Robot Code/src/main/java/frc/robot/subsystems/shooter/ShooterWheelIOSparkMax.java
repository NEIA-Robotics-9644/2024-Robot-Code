package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

public class ShooterWheelIOSparkMax implements ShooterWheelIO {

    private final double maxSpeedRPM = 5000;

    private final CANSparkMax motor;

    private boolean newInput = false;

    private double normalizedVelocity = 0.0;

    public ShooterWheelIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }


    @Override
    public void spinWheel(double normalizedVelocity) {
        this.normalizedVelocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity));
        newInput = true;
    }

    


    @Override
    public double getVelocityPercent() {
        return motor.getEncoder().getVelocity() / maxSpeedRPM;
    }

    @Override
    public void periodic() {
        if (newInput) {
            motor.set(normalizedVelocity);

            newInput = false;

        } else {
            motor.set(0.0);
        } 
    }
}
