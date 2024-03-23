package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    private final ClimberMotorIO[] Climbers = new ClimberMotorIO[2];

    private final boolean[] climberInverted = {false, false};

    

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

        for (int i = 0; i < Climbers.length; i++) {
            Climbers[i].setInverted(climberInverted[i]);
        }

        
    }
    @Override
    public void periodic() {
        
        for (ClimberMotorIO climber : Climbers) {
            climber.periodic();
        }


    }

    
    public void moveClimber(double normalizedVelocity) {
        for (ClimberMotorIO climber : Climbers) {
            climber.spinMotor(normalizedVelocity);
        }
    }

    public double getClimberRotations() {
        double totalRotations = 0;
        for (ClimberMotorIO climber : Climbers) {
            totalRotations += climber.getMotorRotations();
        }
        return totalRotations / Climbers.length;
    }

    public double getClimberSpeed() {
        double totalVelocity = 0;
        for (ClimberMotorIO climber : Climbers) {
            totalVelocity += Math.abs(climber.getMotorVelocityRPM());
        }
        return totalVelocity / Climbers.length;
    }

    public void LClimberMove(double normalizedVelocity)
    {
        Climbers[0].spinMotor(normalizedVelocity);
    }
    public void RClimberMove(double normalizedVelocity)
    {
        Climbers[1].spinMotor(normalizedVelocity);
    }
}
