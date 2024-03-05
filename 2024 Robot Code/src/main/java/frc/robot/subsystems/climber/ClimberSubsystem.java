package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    private final ClimberMotorIO[] Climbers = new ClimberMotorIO[2];

    public ClimberSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ClimberSubsystem(ClimberMotorIO LClimber, ClimberMotorIO RClimber) {

        if (LClimber == null || RClimber == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.Climbers[0] = LClimber;
        this.Climbers[1] = RClimber;
    }
    @Override
    public void periodic() {
        
        for (ClimberMotorIO climber : Climbers) {
            climber.periodic();
        }
    }

    
    public void moveClimber(boolean up) {
        if(up) {
            Climbers[0].spinMotor(1);
            Climbers[1].spinMotor(1);
        } else {
            Climbers[0].spinMotor(-1);
            Climbers[1].spinMotor(-1);
        }
    }
}
