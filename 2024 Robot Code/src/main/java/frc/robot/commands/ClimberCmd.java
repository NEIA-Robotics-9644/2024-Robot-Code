package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.Supplier;

public class ClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    private final boolean up; //false for -, true for +
    
    public ClimberCmd(ClimberSubsystem climberSubsystem, boolean up) {
        
        this.climberSubsystem = climberSubsystem;
        this.up = up;

        addRequirements(climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("ClimberCmd: " + (up ? "up" : "down"));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        climberSubsystem.moveClimber(up);

        System.out.println("ClimberCmd executed: " + (up ? "up" : "down"));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ClimberCmd ended: " + (up ? "up" : "down"));
    }
}