package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;


/*
 * Command to move the shooter to the bottom and reset the angle
 * This command takes over manual control of the shooter angle
 * IT SHOULD NOT BE INTERRUPTED
 * This command should be stopped by the system calling it
 */
public class MoveShooterToBottomAndResetCmd extends Command {

    private final ShooterSubsystem shooter;
    private final double speed;


    public MoveShooterToBottomAndResetCmd(ShooterSubsystem shooter, double speedDegPerSec) {
        this.shooter = shooter;
        this.speed = speedDegPerSec;
    }

    @Override
    public void initialize() {
        shooter.setManualAngleControl(true);
    }

    @Override
    public void execute() {
        
        shooter.setManualAngleVelocityDegPerSec(-speed);
    }

    @Override
    public void end(boolean interrupted) {
        
        shooter.setManualAngleControl(false);
        shooter.resetAngleToBottom();
        shooter.goToSetpoint(0);

        
    }
    
}
