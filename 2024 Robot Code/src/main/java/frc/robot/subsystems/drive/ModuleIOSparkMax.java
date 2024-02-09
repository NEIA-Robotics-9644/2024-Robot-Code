package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ModuleIOSparkMax implements ModuleIO {
    
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;
    private final CANcoder turnAbsoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;

    private final int index;

    public ModuleIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware");
    }

    public ModuleIOSparkMax(int index) {
        this.index = index;
        switch (index) {
            case 0:
                driveSparkMax = new CANSparkMax(2, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(0.047607 * 2 * Math.PI); // MUST BE CALIBRATED
                break;
            case 1:
                driveSparkMax = new CANSparkMax(4, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(10);
                absoluteEncoderOffset = new Rotation2d(0.052734 * 2 * Math.PI); // MUST BE CALIBRATED
                break;
            case 2:
                driveSparkMax = new CANSparkMax(8, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(12);
                absoluteEncoderOffset = new Rotation2d(0.235107 * 2 * Math.PI); // MUST BE CALIBRATED
                break;
            case 3:
                driveSparkMax = new CANSparkMax(6, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(11);
                absoluteEncoderOffset = new Rotation2d(0.220703 * 2 * Math.PI); // MUST BE CALIBRATED
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }
    }

    @Override
    public void periodic() {
        // Log data to SmartDashboard
        SmartDashboard.putNumber("Module " + index + " Drive Velocity", getDriveVelocity());
        SmartDashboard.putNumber("Module " + index + " Turn Absolute Position", turnAbsoluteEncoder.getPosition().getValueAsDouble());
    }

    @Override
    public double getDriveVelocity() {
        return driveSparkMax.getEncoder().getVelocity();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveSparkMax.setVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turnSparkMax.setVoltage(voltage);
    }

    @Override
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d(
            (turnAbsoluteEncoder.getPosition().getValueAsDouble()
                    - Math.floor(turnAbsoluteEncoder.getPosition().getValueAsDouble()))
                * 2.0
                * Math.PI)
        .minus(absoluteEncoderOffset);
    }

    @Override
    public void setDriveBrake(boolean brake) {
        if (brake) {
            driveSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            driveSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }
}
