package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.PhysicalRobotCharacteristics;

public class Module {

    private final ModuleIO io;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
    private final PIDController driveFeedback = new PIDController(1, 0, 0);
    private final PIDController turnFeedback = new PIDController(1, 0, 0);


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

        turnFeedback.setSetpoint(state.angle.getRadians());
        double turnVoltage = turnFeedback.calculate(io.getAbsoluteRotation().getRadians());

        io.setTurnVoltage(turnVoltage);

        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = moduleState.speedMetersPerSecond * Math.cos(turnFeedback.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / PhysicalRobotCharacteristics.kWheelRadiusMeters;
        io.setDriveVoltage(
            driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(io.getDriveVelocity(), velocityRadPerSec));
    }
}