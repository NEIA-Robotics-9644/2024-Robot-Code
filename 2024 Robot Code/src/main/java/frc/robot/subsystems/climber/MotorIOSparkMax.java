package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorIOSparkMax implements MotorIO {
    
    private final CANSparkMax motorSparkMax;
    private final CANcoder turnAbsoluteEncoder;
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
                motorSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Left Flywheel";
                break;
            case 1:
                motorSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Right Flywheel";
                break;
            case 2:
                motorSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Swivel 1";
                break;
            case 3:
                motorSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Swivel 2";
                break;
            case 4:
                motorSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                motorName = "Feed In";
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
        SmartDashboard.putNumber(motorName + " Velocity", getMotorVelocity());
        SmartDashboard.putNumber(motorName + " Turn Absolute Position", turnAbsoluteEncoder.getPosition().getValueAsDouble());
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motorSparkMax.setVoltage(voltage);
    }

    @Override
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d((turnAbsoluteEncoder.getPosition().getValueAsDouble()- Math.floor(turnAbsoluteEncoder.getPosition().getValueAsDouble())) * 2.0 * Math.PI).minus(absoluteEncoderOffset);
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