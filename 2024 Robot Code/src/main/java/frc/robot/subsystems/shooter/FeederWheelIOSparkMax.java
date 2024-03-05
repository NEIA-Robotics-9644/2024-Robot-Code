package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class FeederWheelIOSparkMax implements FeederWheelIO {

    private final CANSparkMax feederMotor;

    
    private boolean newInput = false;

    private double normalizedVelocity = 0.0;

    private double maxSpeedRPM = 5000.0;

    public FeederWheelIOSparkMax(int canID) {
        this.feederMotor = new CANSparkMax(canID, MotorType.kBrushless);
    }

    @Override
    public void spinWheel(double normalizedVelocity) {
        this.normalizedVelocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity));
        newInput = true;
    }

    @Override
    public double getVelocityPercent() {
        return feederMotor.getEncoder().getVelocity() / maxSpeedRPM;
    }

    @Override
    public void periodic() {
        if (newInput) {
            feederMotor.set(normalizedVelocity);
            newInput = false;
        } else {
            feederMotor.set(0.0);
        }
    }

    

    @Override
    public void setBrakeMode(boolean brake) {
        feederMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}
