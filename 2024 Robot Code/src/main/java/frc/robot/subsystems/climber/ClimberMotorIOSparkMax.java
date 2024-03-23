package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class ClimberMotorIOSparkMax implements ClimberMotorIO {
    
    private final CANSparkMax motor;

    private boolean newInput = false;

    private double velocity = 0.0;


    // This depends on the soft limits on the SparkMax being set correctly


    private final double bottomLimitRotations = -10000;

    private final double topLimitRotations = 10000;


    public ClimberMotorIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void spinMotor(double normalizedVelocity) {
        this.velocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity));
        newInput = true;
    }

    @Override
    public double getMotorVelocityRPM() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        if (newInput) {
            
            motor.set(velocity);
            System.out.println("Motor spinning at " + velocity);
            System.out.println("Motor position " +   getMotorRotations());
            newInput = false;
        } else {
            motor.set(0.0);
        }
    }

    @Override
    public double getMotorRotations() {
        return motor.getEncoder().getPosition();
    }


    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }


}