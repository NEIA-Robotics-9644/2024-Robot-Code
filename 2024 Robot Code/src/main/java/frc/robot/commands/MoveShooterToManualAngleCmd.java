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
    private final double shooterWheelSpeed;
    private final double feederWheelSpeed;

    public MoveShooterToManualAngleCmd(ShooterSubsystem shooter, double angle, double shooterWheelSpeed, double feederWheelSpeed) {
        this.shooterSubsystem = shooter;
        this.angle = angle;
        this.shooterWheelSpeed = shooterWheelSpeed;
        this.feederWheelSpeed = feederWheelSpeed;

    }

    public void initialize() {
        
    }

    public void execute() {
        shooterSubsystem.setManualAngleSetpoint(angle, shooterWheelSpeed, feederWheelSpeed);
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }
}
