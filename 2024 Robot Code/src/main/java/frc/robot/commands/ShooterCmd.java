package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.Supplier;

/*
 * This is a command that will be used to drive the robot with a joystick
 */
public class ShooterCmd extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final Supplier<Integer> direction; //- for negative, not for pos
    private final Supplier<Double> speedSupplier;
    
    public ShooterCmd(ShooterSubsystem shooterSubsystem, Supplier<Double> speedSupplier, Supplier<Integer> direction) {
        
        this.shooterSubsystem = shooterSubsystem;
        this.speedSupplier = speedSupplier;
        this.direction = direction;

        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double speed = speedSupplier.get();
        double dir = direction.get();
        shooterSubsystem.shoot(speed * dir);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        shooterSubsystem.shoot(0);
        return false;
    }
}
