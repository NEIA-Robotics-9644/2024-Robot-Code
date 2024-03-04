package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeWheelMotorIOSparkMax implements IntakeWheelMotorIO {

    private final CANSparkMax motor;

    public final double maxSpeedRPM = 5700; // TODO: Find the real value

    
    public IntakeWheelMotorIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
    }

    @Override
    public void runMotorAtPercentVelocity(double percentVelocity) {
        motor.set(percentVelocity);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake) {
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } else {
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
    }


    @Override
    public double getMotorVelocityPercent() {
        return motor.getEncoder().getVelocity() / maxSpeedRPM;
    }


    @Override
    public void periodic() {
        // Log data to SmartDashboard
        // SmartDashboard.putNumber(motorName + " Velocity", getMotorVelocity());
        // SmartDashboard.putNumber(motorName + " Turn Absolute Position", motorSparkMax.getEncoder().getPosition());
    }


    
}
