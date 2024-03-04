package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SpinIntakeWheelsCmd extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final boolean reversed;

    public SpinIntakeWheelsCmd(IntakeSubsystem intakeSubsystem, boolean reversed) {
        this.intakeSubsystem = intakeSubsystem;
        this.reversed = reversed;
    }


    @Override
    public void initialize() {
        System.out.println("Spin Intake Wheels Cmd Initialized");
    }

    @Override
    public void execute() {
        intakeSubsystem.runFeeder(reversed);
        System.out.println("Spin Intake Wheels Cmd Executed");
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println("Spin Intake Wheels Cmd " + (interrupted ? "Interrupted" : "Ended"));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
