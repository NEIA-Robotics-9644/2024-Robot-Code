package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

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
        System.out.println("Shooter being moved" + -speed);
        shooter.setManualAngleVelocityDegPerSec(-speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooter reset");
        shooter.setManualAngleControl(false);
        shooter.resetAngleToBottom();
        shooter.goToSetpoint(0);

        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
