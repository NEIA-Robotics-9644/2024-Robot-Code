package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;


/*
 * This is a command that will be used to move the shooter to a setpoint
 * Since the shooter continues to move to the setpoint until it reaches it, we can set it and then end the command
 */
public class MoveShooterToSetpointCmd extends Command {
    private final ShooterSubsystem shooter;
    private final double setpoint;

    public MoveShooterToSetpointCmd(ShooterSubsystem shooter, double setpoint) {
        this.shooter = shooter;
        this.setpoint = setpoint;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterAngleDeg(setpoint);
        System.out.println("Moving shooter to setpoint " + setpoint + " degrees");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
