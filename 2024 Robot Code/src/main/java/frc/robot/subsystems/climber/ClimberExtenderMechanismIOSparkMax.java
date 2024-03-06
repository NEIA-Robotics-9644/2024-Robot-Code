package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
public class ClimberExtenderMechanismIOSparkMax implements ClimberExtenderMechanismIO {
    
    private final double extendedSetpoint = 0.0;
    private final double retractedSetpoint = 180.0;

    private final double extendBaseSpeedDegPerSec = 1.0;
    private final PIDController extendFeedback = new PIDController(0.1, 0.0, 0.0);  // TODO: Tune these
    private final double extendMaxSpeedDegPerSec = 360.0;

    private final double retractBaseSpeedDegPerSec = 3.0;
    private final PIDController retractFeedback = new PIDController(0.1, 0.0, 0.0);  // TODO: Tune these
    private final double retractMaxSpeedDegPerSec = 360.0;
    
    private final double encoderOffsetDeg = 0.0;

    private final CANSparkMax motor;
    

    private boolean extended = false;

    

    public ClimberExtenderMechanismIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }


    public ClimberExtenderMechanismIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setExtended(boolean extended) {

        // If the state changed, then reset the feedback controllers
        if (this.extended != extended) {
            extendFeedback.reset();
            retractFeedback.reset();
        }
        
        this.extended = extended;

        
    }

    @Override
    public void periodic() {
        if (extended) {
            extendFeedback.setSetpoint(extendedSetpoint);

            motor.set(extendFeedback.calculate(getAngle(), extendedSetpoint) + -(extendBaseSpeedDegPerSec / extendMaxSpeedDegPerSec));
        } else {
            retractFeedback.setSetpoint(retractedSetpoint);

            motor.set(retractFeedback.calculate(getAngle(), retractedSetpoint) + (retractBaseSpeedDegPerSec / retractMaxSpeedDegPerSec));
        }
    }

    @Override
    public boolean getExtended() {
        return extended;
    }


    @Override
    public double getAngle() {
        return (motor.getEncoder().getPosition() * 360) + encoderOffsetDeg;
    }
}