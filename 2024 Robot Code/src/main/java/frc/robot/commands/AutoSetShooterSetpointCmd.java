package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.Supplier;


/*
 * Shoot when the shooter wheels are at a high enough speed.
 * Doesn't require the shooter subsystem, but it does use it.
 * This is so that the command can be run in parallel with other commands that use the shooter subsystem.
 */
public class AutoSetShooterSetpointCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;

    private final int setpointIndex;
    
    /*
     * @param shooterSubsystem The shooter subsystem to use
     * @param percentOffToOnThreshold The percent speed at which to turn the feeder on if it is off
     * @param percentOnToOffThreshold The percent speed at which to turn the feeder off if it is on
     */
    public AutoSetShooterSetpointCmd(ShooterSubsystem shooter, int setpoint) {
        this.shooterSubsystem = shooter;
        this.setpointIndex = setpoint;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.goToSetpoint(setpointIndex);
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
