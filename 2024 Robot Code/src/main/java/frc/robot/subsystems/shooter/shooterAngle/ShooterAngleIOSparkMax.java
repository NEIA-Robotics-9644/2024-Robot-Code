package frc.robot.subsystems.shooter.shooterAngle;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterAngleIOSparkMax implements ShooterAngleIO {


    
    private final double bottomLimitDeg = 0.0;
    private final double topLimitDeg = 100.0;
    
    private final double encoderOffsetDeg = 0.0;

    private final double startingOffsetDeg = 75.0;

    private final boolean encoderReversed = false;


    
    private final CANSparkMax leftAngleMotor;
    private final CANSparkMax rightAngleMotor;

    private final boolean leftReversed = false;
    private final boolean rightReversed = true;



    private double setpointDeg = 0.0;



    private double encoderReadingRotationsToAngleDeg = 360.0 / 80.0;

    private final double maxSpeedDegPerSec = 50.0;



    private boolean manualControl = false;

    

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.1, 0.0);  // TODO: Tune these
    private PIDController feedback = new PIDController(0.008, 0.0, 0.0);  // TODO: Tune these


    public ShooterAngleIOSparkMax() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterAngleIOSparkMax(int leftAngleMotorCanID, int rightAngleMotorCanID) {
        this.leftAngleMotor = new CANSparkMax(leftAngleMotorCanID, CANSparkMax.MotorType.kBrushless);
        this.rightAngleMotor = new CANSparkMax(rightAngleMotorCanID, CANSparkMax.MotorType.kBrushless);
        
        
        this.leftAngleMotor.setInverted(leftReversed);
        

        // Set the right motor to follow the left motor, but keep in mind that they might be reversed
        this.rightAngleMotor.follow(this.leftAngleMotor, leftReversed != rightReversed);

        this.leftAngleMotor.setSmartCurrentLimit(40);
        this.rightAngleMotor.setSmartCurrentLimit(40);
        this.leftAngleMotor.setOpenLoopRampRate(0.5);
        this.rightAngleMotor.setOpenLoopRampRate(0.5);


        this.leftAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.rightAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        

        
        

        this.leftAngleMotor.burnFlash();
        this.rightAngleMotor.burnFlash();

        this.feedback.setTolerance(0.1);


        leftAngleMotor.getEncoder().setPosition((bottomLimitDeg + encoderOffsetDeg + startingOffsetDeg) / encoderReadingRotationsToAngleDeg);
        
    }



    @Override
    public void setAngleDeg(double setpoint) {

        // Clamp the setpoint between the higher and lower limits
        double setpointClamped = Math.max(bottomLimitDeg, Math.min(topLimitDeg, setpoint));
        feedback.setSetpoint(setpointClamped);
    }

    @Override
    public void periodic() {

        if (!manualControl) {
            // Calculate the feedforward
            double feedforwardOutput = feedforward.calculate(setpointDeg);

            // Calculate the feedback
            double feedbackOutput = feedback.calculate(getAngleDeg());

            double output = feedforwardOutput + feedbackOutput;

            
            // Clamp so it is under the max speed
            if (output > maxSpeedDegPerSec) {
                output = maxSpeedDegPerSec;
            } else if (output < -maxSpeedDegPerSec) {
                output = -maxSpeedDegPerSec;
            }
            

            // Set the motor output
            leftAngleMotor.set(output);

        }
        


        
    }

    @Override
    public double getVelocityPercent() {
        return (leftAngleMotor.getEncoder().getVelocity() * (1/80.0) * (encoderReversed ? -1 : 1) * 360.0 * (1/60.0)) / maxSpeedDegPerSec;
    }


    @Override
    public double getAngleDeg() {
        
        double angle = leftAngleMotor.getEncoder().getPosition() * encoderReadingRotationsToAngleDeg * (encoderReversed ? -1 : 1);
        return angle + encoderOffsetDeg;
    }


    @Override
    public void setManualControl(boolean enabled) {
        manualControl = enabled;
    }

    @Override
    public boolean manualControlEnabled() {
        return manualControl;
    }

    @Override
    public void setManualVelocity(double normalizedVelocity) {
        if (manualControl) {
            leftAngleMotor.set(normalizedVelocity);
        }
    }

    
    @Override
    public boolean atBottom() {
        // TODO: Make this better
        return getAngleDeg() <= bottomLimitDeg + 0.1;
    }



    @Override
    public boolean atTop() {
        // TODO: Make this better
        return getAngleDeg() >= topLimitDeg - 0.1;
    }

    @Override
    public void resetAngleToBottom() {
        
        leftAngleMotor.getEncoder().setPosition((bottomLimitDeg + encoderOffsetDeg) / encoderReadingRotationsToAngleDeg);
    }


    @Override
    public double getTopAngleDeg() {
        return topLimitDeg;
    }

    @Override
    public double getBottomAngleDeg() {
        return bottomLimitDeg;
    }

}