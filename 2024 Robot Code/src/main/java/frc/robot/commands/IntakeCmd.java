package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.function.Supplier;

/*
 * This is a command that will be used to drive the robot with a joystick
 */
public class IntakeCmd extends Command {

    private final IntakeSubsystem IntakeSubsystem;
    private final Supplier<Boolean> deploySupplier;
    private final Supplier<Double> feederSupplier; //what does this do
    
    public IntakeCmd(IntakeSubsystem IntakeSubsystem, Supplier<Double> feederSupplier, Supplier<Boolean> deploySupplier) {
        
        this.IntakeSubsystem = IntakeSubsystem;
        this.feederSupplier = feederSupplier;
        this.deploySupplier = deploySupplier;

        addRequirements(IntakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double feederValue = feederSupplier.get();
        boolean deployValue = deploySupplier.get();
        IntakeSubsystem.deploy(deployValue);
        IntakeSubsystem.runFeeder(feederValue);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double feederValue = feederSupplier.get();
        boolean deployValue = deploySupplier.get();
        IntakeSubsystem.deploy(deployValue);
        IntakeSubsystem.runFeeder(feederValue);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        double feederValue = 0;
        boolean deployValue = false;

        IntakeSubsystem.deploy(deployValue);
        IntakeSubsystem.runFeeder(feederValue);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
