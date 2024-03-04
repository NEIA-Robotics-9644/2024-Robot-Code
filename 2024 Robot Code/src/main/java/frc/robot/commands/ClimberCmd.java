package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.Supplier;

public class ClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    private final Supplier<Boolean> direction; //false for -, true for +
    private final Supplier<Double> speedSupplier;
    
    public ClimberCmd(ClimberSubsystem climberSubsystem, Supplier<Double> speedSupplier, Supplier<Boolean> direction) {
        
        this.climberSubsystem = climberSubsystem;
        this.speedSupplier = speedSupplier;
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

        double speed = speedSupplier.get();
        boolean dir = direction.get();
        climberSubsystem.move(speed, dir);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
}