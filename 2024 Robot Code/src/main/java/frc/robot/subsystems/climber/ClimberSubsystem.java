package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    private final Motor[] Climbers = new Motor[2];

    public ClimberSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ClimberSubsystem(MotorIO LClimber, MotorIO RClimber) {

        if (LClimber == null || RClimber == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.Climbers[0] = new Motor(LClimber);
        this.Climbers[1] = new Motor(RClimber);
    }
    @Override
    public void periodic() {
        Climbers[0].periodic();
        Climbers[1].periodic();
    }
    public void move(double voltage, boolean direction)
    {
        if(direction)
        {
            Climbers[0].setActive(voltage);
            Climbers[1].setActive(voltage);
        }
        else if(direction == false)
        {
            Climbers[0].setActive(-voltage);
            Climbers[1].setActive(-voltage);
        }
    }
}
