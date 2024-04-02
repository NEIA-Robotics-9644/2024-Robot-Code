package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunSourceIntakeCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;

    private boolean oldNoteDetectedState = false;

    private boolean shutoffTimerEnabled = false;

    private final double kShutoffDelay = 0.3;  // Seconds

    private double shutoffTime = 0.0;  // An absolute time at which we should shut off

    public RunSourceIntakeCmd(ShooterSubsystem shooterSubsystem) {

        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        //System.out.println("Run Source Intake Cmd Initialized");
        shutoffTimerEnabled = false;
        oldNoteDetectedState = false;

    }

    @Override
    public void execute() {

        // Run the shooter wheels backwards
        //shooterSubsystem.spinShooterWheels(true);
        
        // Run the feeder wheel backwards
        shooterSubsystem.spinFeederWheel(true);


        //System.out.println("Run Source Intake Cmd Executed");
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("Run Source Intake Cmd " + (interrupted ? "Interrupted" : "Ended"));
    }

    @Override
    public boolean isFinished() {


        return false;
        /*
        if (shutoffTimerEnabled) {
            return Timer.getFPGATimestamp() > shutoffTime;
        }

        
        // Only stop if the note is detected for the first time since the command was started
        boolean noteDetected = shooterSubsystem.noteDetected();

        

        if (noteDetected && !oldNoteDetectedState) {
            oldNoteDetectedState = true;

            // Get the current time from FPGATimestamp
            shutoffTimerEnabled = true;
            shutoffTime = Timer.getFPGATimestamp() + kShutoffDelay;
            
        } else {
            oldNoteDetectedState = noteDetected;
        }

        return false;

        */
    }
    
}
