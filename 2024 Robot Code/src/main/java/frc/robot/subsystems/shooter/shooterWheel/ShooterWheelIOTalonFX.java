package frc.robot.subsystems.shooter.shooterWheel;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShooterWheelIOTalonFX implements ShooterWheelIO {

    private final double maxSpeedRPM = 5000;

    private final TalonFX motor;

    private boolean newInput = false;

    private double normalizedVelocity = 0.0;


    public ShooterWheelIOTalonFX(int canID) {
        this.motor = new TalonFX(canID);
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
        return (motor.getVelocity().getValueAsDouble() * 60) / maxSpeedRPM;
    }

    @Override
    public void periodic() {
        if (newInput) {
            motor.set(normalizedVelocity);

            newInput = false;

        } else {
            motor.set(0.0);
        } 

        Shuffleboard.getTab("Current").addDouble("Left Shooter Motor Current", () -> motor.getSupplyCurrent().getValueAsDouble());
        
    }
}
