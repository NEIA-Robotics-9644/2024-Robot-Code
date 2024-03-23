package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;


/*
 * Shoot when the shooter wheels are at a high enough speed.
 * Doesn't require the shooter subsystem, but it does use it.
 * This is so that the command can be run in parallel with other commands that use the shooter subsystem.
 */
public class ShootWhenReadyCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;

    private final double percentOffToOnThreshold;
    private final double percentOnToOffThreshold;

    private boolean running = false;

    /*
     * @param shooterSubsystem The shooter subsystem to use
     * @param percentOffToOnThreshold The percent speed at which to turn the feeder on if it is off
     * @param percentOnToOffThreshold The percent speed at which to turn the feeder off if it is on
     */
    public ShootWhenReadyCmd(ShooterSubsystem shooterSubsystem, double percentOffToOnThreshold, double percentOnToOffThreshold) {
        this.shooterSubsystem = shooterSubsystem;
        this.percentOffToOnThreshold = percentOffToOnThreshold;
        this.percentOnToOffThreshold = percentOnToOffThreshold;
    }

    @Override
    public void initialize() {
        //System.out.println("Shoot When Ready Cmd Initialized");
    }

    @Override
    public void execute() {
        double speed = shooterSubsystem.getShooterWheelsSpeedPercent();

        if (speed > percentOffToOnThreshold && !running) {
            running = true;
        } else if (speed < percentOnToOffThreshold && running) {
            running = false;
        }

        // Make sure not to override other commands that might be using the feeder wheel at the same time
        // This is why we don't stop the feeder wheel even if the shooter wheels are off

        if (running) {
            shooterSubsystem.spinFeederWheel(false);
        }

        //System.out.println("Shoot When Ready Cmd Executed" + "   Shooter Wheels at " + speed + "Percent Speed   " + (running ? "Feeder On" : "Feeder Off"));
    }

    @Override
    public void end(boolean interrupted) {
       //System.out.println("Shoot When Ready Cmd " + (interrupted ? "Interrupted" : "Ended"));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
