package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberMotorIOSparkMax implements ClimberMotorIO {
    
    private final CANSparkMax motor;
   
    public ClimberMotorIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
    }

    @Override
    public void spinMotor(double normalizedVelocity) {
        motor.set(normalizedVelocity);
    }

    @Override
    public double getMotorRotations() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getMotorVelocityRPM() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        // No periodic behavior needed
    }
}