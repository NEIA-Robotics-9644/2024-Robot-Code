package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorIOSparkMax implements MotorIO {
    
    private final CANSparkMax motorSparkMax;
    private final Rotation2d absoluteEncoderOffset;
    private final String motorName;

    private final int index;

    public MotorIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware");
    }

    public MotorIOSparkMax(int index) {
        this.index = index;
        switch (index) {
            case 0:
                motorSparkMax = new CANSparkMax(26, MotorType.kBrushless);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Feeder";
                break;
            case 1:
                motorSparkMax = new CANSparkMax(27, MotorType.kBrushless);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Deployer";
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }
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