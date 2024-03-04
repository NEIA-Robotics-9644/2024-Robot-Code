package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SpinShooterWheelsCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;

    public SpinShooterWheelsCmd(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Spin Shooter Wheels Cmd Initialized");
    }

    @Override
    public void execute() {
        shooterSubsystem.spinShooterWheels(false);
        System.out.println("Spin Shooter Wheels Cmd Executed");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Spin Shooter Wheels Cmd " + (interrupted ? "Interrupted" : "Ended"));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
