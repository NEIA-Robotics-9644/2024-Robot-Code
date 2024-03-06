package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

public class ClimberExtenderMechanism {

    private final ClimberExtenderMechanismIO io;

    private final PIDController turnFeedback = new PIDController(5, 0, 0);


    public ClimberExtenderMechanism() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ClimberExtenderMechanism(ClimberExtenderMechanismIO io) {
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

    public void extend(boolean extended) {
        io.setExtended(extended);
    }
    
}
