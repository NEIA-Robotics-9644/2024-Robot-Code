package frc.robot.commands;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SpinShooterWheelsCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;

    private final CommandXboxController controller;

    public SpinShooterWheelsCmd(ShooterSubsystem shooterSubsystem, CommandXboxController controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.controller = controller;
    }

    

    @Override
    public void initialize() {
        //System.out.println("Spin Shooter Wheels Cmd Initialized");
    }

    @Override
    public void execute() {
        shooterSubsystem.spinShooterWheels(false);
        //System.out.println("Spin Shooter Wheels Cmd Executed");
        if (shooterSubsystem.getShooterWheelsSpeedPercent() > 1.15){
        controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
        controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);
    }
    else {
        controller.getHID().setRumble(RumbleType.kLeftRumble, 0);
        controller.getHID().setRumble(RumbleType.kRightRumble, 0);
    }
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("Spin Shooter Wheels Cmd " + (interrupted ? "Interrupted" : "Ended"));
        controller.getHID().setRumble(RumbleType.kLeftRumble, 0);
        controller.getHID().setRumble(RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
