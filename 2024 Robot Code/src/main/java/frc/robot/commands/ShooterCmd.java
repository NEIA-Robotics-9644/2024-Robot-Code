package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.Supplier;

public class ShooterCmd extends Command {
    public static final boolean FORWARD_DIRECTION_REVERSED = true;

    private final ShooterSubsystem shooterSubsystem;

    private final Supplier<Integer> direction; //- for negative, not for pos
    private final Supplier<Double> speedSupplier;
    private final Supplier<Double> feederSupplier;
    
    public ShooterCmd(ShooterSubsystem shooterSubsystem, Supplier<Double> speedSupplier, Supplier<Integer> direction, Supplier<Double> feederSupplier) {
        
        this.shooterSubsystem = shooterSubsystem;
        this.speedSupplier = speedSupplier;
        this.direction = direction;
        this.feederSupplier = feederSupplier;

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
        double feederSpeed = feederSupplier.get();
        //shooterSubsystem.shoot(speed * dir * (FORWARD_DIRECTION_REVERSED ? -1 : 1));
        //shooterSubsystem.setFeederSpeed(feederSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
