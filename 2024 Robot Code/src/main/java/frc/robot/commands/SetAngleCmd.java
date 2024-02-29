package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.SwerveDriveSubsystem;

public class SetAngleCmd extends Command{

    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<Double> rotationalSupplier;

    public SetAngleCmd(SwerveDriveSubsystem driveSubsystem, Supplier<Double> rotationalSupplier) {
        
        this.driveSubsystem = driveSubsystem;
        this.rotationalSupplier = rotationalSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        

        double deadband = 0.1;
        
        ChassisSpeeds speed = new ChassisSpeeds(0,0, MathUtil.applyDeadband(1, deadband, 1));
        driveSubsystem.turnCenter(speed, timevar, lOrRbool);
    }
}
