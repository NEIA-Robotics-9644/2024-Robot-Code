package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeCmd extends Command {


    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;


    public IntakeCmd(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }


    @Override
    public void initialize() {
        //System.out.println("Spin Intake Wheels Cmd Initialized");
    }

    @Override
    public void execute() {
        intakeSubsystem.runFeeder(false);
        intakeSubsystem.setExtended(true);
        shooterSubsystem.spinFeederWheel(false);
        //System.out.println("Spin Intake Wheels Cmd Executed");
    }


    @Override
    public void end(boolean interrupted) {
        //System.out.println("Spin Intake Wheels Cmd " + (interrupted ? "Interrupted" : "Ended"));
        intakeSubsystem.setExtended(false);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.noteDetected();
    }
    
}
