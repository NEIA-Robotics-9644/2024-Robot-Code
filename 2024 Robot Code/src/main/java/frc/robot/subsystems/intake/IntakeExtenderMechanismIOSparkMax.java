package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;


public class IntakeExtenderMechanismIOSparkMax implements IntakeExtenderMechanismIO {

    // IMPORTANT: THE SOFT LIMITS ARE NOT SET HERE.  THEY SHOULD BE FOUND IN REV HARDWARE CLIENT AND THEN COPIED HERE
    private final double extendedSetpoint = 0.0;
    private final double retractedSetpoint = 90.0;

    // The motor is fighting gravity when it is retracting, so it needs to be faster
    private final double retractBaseSpeedDegPerSec = 3.0;
    private final PIDController retractFeedback = new PIDController(0.3, 0.0, 0.0);  // TODO: Tune these
    private final double retractMaxSpeedDegPerSec = 180.0;

    private final double extendBaseSpeedDegPerSec = 1.0;
    private final PIDController extendFeedback = new PIDController(0.1, 0.0, 0.0);  // TODO: Tune these
    private final double extendMaxSpeedDegPerSec = 360.0;
    
    private final double encoderOffsetDeg = 0.0;

    private final CANSparkMax motor;

    

    private final boolean motorReversed = false;

    private boolean extended = false;

    

    public IntakeExtenderMechanismIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }


    public IntakeExtenderMechanismIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(motorReversed);
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
        
        
        // Try to move the motor to the the setpoint, and rely on internal soft limits to prevent overtravel
        if (extended) {
            extendFeedback.setSetpoint(extendedSetpoint);

            motor.set(extendFeedback.calculate(getAngleDeg(), extendedSetpoint) + -(extendBaseSpeedDegPerSec / extendMaxSpeedDegPerSec));
        } else {
            retractFeedback.setSetpoint(retractedSetpoint);

            motor.set(retractFeedback.calculate(getAngleDeg(), retractedSetpoint) + (retractBaseSpeedDegPerSec / retractMaxSpeedDegPerSec));
        }
    }

    @Override
    public boolean getExtended() {
        return extended;
    }


    @Override
    public double getAngleDeg() {
        return (motor.getEncoder().getPosition() * 360) + encoderOffsetDeg;
    }





}
    