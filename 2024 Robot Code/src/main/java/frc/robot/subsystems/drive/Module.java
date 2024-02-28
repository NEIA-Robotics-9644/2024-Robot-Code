package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.PhysicalRobotCharacteristics;
import edu.wpi.first.math.MathUtil;

public class Module {

    private final ModuleIO io;

    private final PIDController turnFeedback = new PIDController(5, 0, 0);


    public Module() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public Module(ModuleIO io) {
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

    public void drive(SwerveModuleState moduleState) {
        // Calculate module state
        SwerveModuleState state = SwerveModuleState.optimize(moduleState, io.getAbsoluteRotation());

        turnFeedback.setSetpoint(MathUtil.angleModulus(state.angle.getRadians()));
        double turnVoltage = turnFeedback.calculate(MathUtil.angleModulus(io.getAbsoluteRotation().getRadians()));

        io.setTurnVoltage(turnVoltage);
        // Run drive controller
        io.setDriveVoltage(state.speedMetersPerSecond);
    }

    public void turn(SwerveModuleState moduleState, double angle, double time) {
        // Calculate module state
        SwerveModuleState state = SwerveModuleState.optimize(moduleState, io.getAbsoluteRotation());

        turnFeedback.setSetpoint(angle);
        double turnVoltage = turnFeedback.calculate(MathUtil.angleModulus(io.getAbsoluteRotation().getRadians()));

        io.setTurnVoltage(turnVoltage);
        // Run drive controller
        io.setDriveVoltage(state.speedMetersPerSecond);
    }
    public void breakAll()
    {
        io.setTurnVoltage(0);
        io.setDriveVoltage(0);

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(io.getDriveVelocity() / 60.0 * Math.PI * 2.0 * PhysicalRobotCharacteristics.kWheelRadiusMeters * 0.02, io.getAbsoluteRotation());
    }
}
