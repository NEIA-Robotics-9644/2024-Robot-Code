package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    private final ClimberExtenderMechanism[] Climbers = new ClimberExtenderMechanism[2];

    public ClimberSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ClimberSubsystem(ClimberExtenderMechanismIO LClimber, ClimberExtenderMechanismIO RClimber) {

        if (LClimber == null || RClimber == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.Climbers[0] = new ClimberExtenderMechanism(LClimber);
        this.Climbers[1] = new ClimberExtenderMechanism(RClimber);
    }
    @Override
    public void periodic() {
        Climbers[0].periodic();
        Climbers[1].periodic();
    }
    public void move(boolean direction)
    {
        Climbers[0].extend(direction);
        Climbers[1].extend(direction);
    }
}
