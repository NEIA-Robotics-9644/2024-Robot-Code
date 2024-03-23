package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class MoveShooterToSetpointCmd extends Command {
    

    private final ShooterSubsystem shooterSubsystem;
    private final int setpoint;

    public MoveShooterToSetpointCmd(ShooterSubsystem shooter, int setpoint) {
        this.shooterSubsystem = shooter;
        this.setpoint = setpoint;
    }

    public void initialize() {

    }

    public void execute() {
        shooterSubsystem.goToSetpoint(setpoint);
    }

    public boolean isFinished() {
        return false;
    }
}
