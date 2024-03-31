package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.climberMotor.ClimberMotorIO;

public class ClimberSubsystem extends SubsystemBase{

    private final ClimberMotorIO leftClimber;
    private final ClimberMotorIO rightClimber;

    private final boolean leftClimberReversed = false;
    private final boolean rightClimberReversed = false;

    

    public ClimberSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ClimberSubsystem(ClimberMotorIO leftClimber, ClimberMotorIO rightClimber) {

        if (leftClimber == null || rightClimber == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        this.leftClimber = leftClimber;
        this.rightClimber = rightClimber;

        this.leftClimber.setInverted(leftClimberReversed);
        this.rightClimber.setInverted(rightClimberReversed);

        
    }
    @Override
    public void periodic() {
        
        leftClimber.periodic();
        rightClimber.periodic();

        


    }

    
    public void moveClimber(double normalizedVelocity) {
        leftClimber.spinMotor(normalizedVelocity);
        rightClimber.spinMotor(normalizedVelocity);
    }

    public double getClimberRotations() {
        return (leftClimber.getMotorRotations() + rightClimber.getMotorRotations()) / 2;
    }

    public double getClimberSpeed() {
        return (Math.abs(leftClimber.getMotorVelocityRPM())
             + Math.abs(rightClimber.getMotorVelocityRPM())
            ) / 2;
    }

    public void moveLeftClimber(double normalizedVelocity) {
        leftClimber.spinMotor(normalizedVelocity);
    }

    public void moveRightClimber(double normalizedVelocity) {
        rightClimber.spinMotor(normalizedVelocity);
    }
}
