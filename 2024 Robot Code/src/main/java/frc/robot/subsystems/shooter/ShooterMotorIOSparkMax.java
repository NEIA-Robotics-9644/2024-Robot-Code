package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterMotorIOSparkMax implements ShooterMotorIO {
    
    private final CANSparkMax motorSparkMax;
    private final Rotation2d absoluteEncoderOffset;
    private final String motorName;

    private final ShooterMotorType type;

    private final double maxSpeedRPM;


    public enum ShooterMotorType {
        LEFT_FLYWHEEL, RIGHT_FLYWHEEL, FEEDER
    }

    public ShooterMotorIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware");
    }

    public ShooterMotorIOSparkMax(ShooterMotorType type) {
        this.type = type;
        switch (type) {
            case LEFT_FLYWHEEL:
                motorSparkMax = new CANSparkMax(21, MotorType.kBrushless);
                absoluteEncoderOffset = new Rotation2d(0.0 * 2 * Math.PI); // MUST BE CALIBRATED
                maxSpeedRPM = 5000; // MUST BE CALIBRATED
                motorName = "Left Flywheel";
                break;
            case RIGHT_FLYWHEEL:
                motorSparkMax = new CANSparkMax(22, MotorType.kBrushless);
                absoluteEncoderOffset = new Rotation2d(0.0 * 2 * Math.PI); // MUST BE CALIBRATED
                maxSpeedRPM = 5000; // MUST BE CALIBRATED
                motorName = "Right Flywheel";
                break;
            case FEEDER:
                motorSparkMax = new CANSparkMax(23, MotorType.kBrushless);
                absoluteEncoderOffset = new Rotation2d(0.0 * 2 * Math.PI); // MUST BE CALIBRATED
                maxSpeedRPM = 5000; // MUST BE CALIBRATED
                motorName = "Feed In";
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }
    }
    @Override
    public double getMotorVelocityRPM() {
        return motorSparkMax.getEncoder().getVelocity();
    }


    @Override
    public double getMotorPercentSpeed() {
        return getMotorVelocityRPM() / maxSpeedRPM;
    }


    @Override
    public void periodic() {
        // Log data to SmartDashboard
        
    }

    /*
     * Use setMotorVelocity instead
     */
    @Override
    public void setMotorVoltage(double voltage) {
        motorSparkMax.setVoltage(voltage);
    }

    /*
     * Get the maximum voltage the motor can be set to
     * Remember that setMotorVelocity() exists
     */
    @Override
    public double getMaxMotorVoltage() {
        return 12.0;
    }


    /*
     * Set the velocity of the motor
     * Normalized value between -1 and 1
     */
    @Override
    public void setMotorVelocity(double normalizedVelocity) {
        motorSparkMax.set(normalizedVelocity);
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