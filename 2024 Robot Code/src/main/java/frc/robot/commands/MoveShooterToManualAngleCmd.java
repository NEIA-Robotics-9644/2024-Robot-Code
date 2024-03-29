package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/*
 * Command to move the shooter to a setpoint
 * Never ends, so it should be stopped by the system calling it
 */
public class MoveShooterToManualAngleCmd extends Command {
    

    private final ShooterSubsystem shooterSubsystem;
    private final double angle;

    public MoveShooterToManualAngleCmd(ShooterSubsystem shooter, double angle) {
        this.shooterSubsystem = shooter;
        this.angle = angle;

    }

    public void initialize() {
        
    }

    public void execute() {
        shooterSubsystem.setManualAngleSetpoint(angle);
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }
}
