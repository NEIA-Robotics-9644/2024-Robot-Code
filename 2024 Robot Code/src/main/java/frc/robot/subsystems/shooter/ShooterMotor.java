package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

public class ShooterMotor {

    private final ShooterMotorIO io;

    private final PIDController turnFeedback = new PIDController(5, 0, 0);


    public ShooterMotor() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterMotor(ShooterMotorIO io) {
        // Check for null hardware
        if (io == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize hardware
        this.io = io;

        // Configure turn PID
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }


    /*
     * Periodic function to be called in the robot's periodic function
     * SHOULD BE CALLED ONCE PER CYCLE
     */
    public void periodic() {
        io.periodic();
    }

    /*
     * Set the velocity of the motor
     * Normalized value between -1 and 1
     */
    public void setMotorVelocity(double normalizedVelocity) {
        io.setMotorVelocity(normalizedVelocity);
    }

    /*
     * Get the velocity of the motor
     * In RPM
     */
    public double getMotorVelocityRPM() {
        return io.getMotorVelocityRPM();
    }

    
    /*
     * Get the percent speed of the motor
     * Normalized value between 0 and 1
     * Based on the motor IO's internal max speed
     */
    public double getMotorPercentSpeed() {
        return io.getMotorPercentSpeed();
    }

    
    public void setBrake(boolean braking) {
        io.setBrake(braking);
    }

    /*
    public void setAngle(double angle) {

        turnFeedback.setSetpoint(MathUtil.angleModulus(angle));
        double voltage = turnFeedback.calculate(MathUtil.angleModulus(io.getAbsoluteRotation().getRadians()));

        io.setMotorVoltage(voltage);
    }
    */
}
