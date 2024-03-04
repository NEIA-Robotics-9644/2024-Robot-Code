package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeExtenderMotorIOSparkMax implements IntakeExtenderMotorIO {
    
    private final CANSparkMax motorSparkMax;
    private final Rotation2d absoluteEncoderOffset;

    public IntakeExtenderMotorIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware");
    }

    public IntakeExtenderMotorIOSparkMax(int canID) {
        motorSparkMax = new CANSparkMax(canID, MotorType.kBrushless);
        motorSparkMax.restoreFactoryDefaults();
        motorSparkMax.setIdleMode(IdleMode.kBrake);
        absoluteEncoderOffset = new Rotation2d(0);
    }
    @Override
    public double getMotorVelocity() {
        return motorSparkMax.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        // Log data to SmartDashboard
        // SmartDashboard.putNumber(motorName + " Velocity", getMotorVelocity());
        // SmartDashboard.putNumber(motorName + " Turn Absolute Position", motorSparkMax.getEncoder().getPosition());
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motorSparkMax.setVoltage(voltage);
    }

    @Override
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d((motorSparkMax.getEncoder().getPosition()- Math.floor(motorSparkMax.getEncoder().getPosition()))* 2.0 * Math.PI).minus(absoluteEncoderOffset);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake) {
            motorSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            motorSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }
}