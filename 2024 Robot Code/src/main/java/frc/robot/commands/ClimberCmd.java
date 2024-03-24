package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.Supplier;


/*
 * Command to move the climber
 * This command takes in an input and moves the both climber arms at that velocity
 */
public class ClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    private final Supplier<Double> velocity;
    
    public ClimberCmd(ClimberSubsystem climberSubsystem, Supplier<Double> velocity) {
        
        this.climberSubsystem = climberSubsystem;
        this.velocity = velocity;

        addRequirements(climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //System.out.println("ClimberCmd initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        climberSubsystem.moveClimber(velocity.get());
        //System.out.println("ClimberCmd executed");

    }
}