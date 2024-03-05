package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.Supplier;

public class ClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    private final Supplier<Boolean> direction; //false for -, true for +
    
    public ClimberCmd(ClimberSubsystem climberSubsystem, Supplier<Boolean> direction) {
        
        this.climberSubsystem = climberSubsystem;
        this.direction = direction;

        addRequirements(climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        climberSubsystem.moveClimber(direction.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
}