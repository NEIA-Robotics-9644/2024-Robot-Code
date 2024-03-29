package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hook.HookSubsystem;

import java.util.function.Supplier;


/*
 * Command to move the climber
 * This command takes in an input and moves the both climber arms at that velocity
 */
public class MoveHookCmd extends Command {

    private final HookSubsystem hookSubsystem;

    private final Supplier<Double> velocity;

    public MoveHookCmd(HookSubsystem hookSubsystem, Supplier<Double> velocity) {
        
        this.hookSubsystem = hookSubsystem;
        this.velocity = velocity;

        addRequirements(hookSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("HookCmd initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        hookSubsystem.moveHook(velocity.get());
        System.out.println("HookCmd executed");

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("HookCmd ended");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


}