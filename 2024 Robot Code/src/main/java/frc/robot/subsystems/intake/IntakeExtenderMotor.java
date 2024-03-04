package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class IntakeExtenderMotor {

    private final IntakeExtenderMotorIO io;

    private final PIDController turnFeedback = new PIDController(5, 0, 0);


    public IntakeExtenderMotor() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public IntakeExtenderMotor(IntakeExtenderMotorIO io) {
        // Check for null hardware
        if (io == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize hardware
        this.io = io;

        // Configure turn PID
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.periodic();
    }

    public void setActive(double voltage) {
        io.setMotorVoltage(voltage);
    }

    public void setAngle(double angle) {

        turnFeedback.setSetpoint(MathUtil.angleModulus(angle));
        double voltage = turnFeedback.calculate(MathUtil.angleModulus(io.getAbsoluteRotation().getRadians()));

        io.setMotorVoltage(voltage);
    }
    
}
