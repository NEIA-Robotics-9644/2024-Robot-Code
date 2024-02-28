package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.SwerveDriveSubsystem;

public class SetAngleCmd extends Command{

    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<Boolean> lOrR;
    private final Supplier<Double> time;

    public SetAngleCmd(SwerveDriveSubsystem driveSubsystem, Supplier<Boolean> lOrR, Supplier<Double> time) {
        
        this.driveSubsystem = driveSubsystem;
        this.lOrR = lOrR;
        this.time = time;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        boolean lOrRbool = lOrR.get();
        double timevar = time.get();

        double deadband = 0.1;
        
        ChassisSpeeds speed = new ChassisSpeeds(0,0, MathUtil.applyDeadband(1, deadband, 1));
        driveSubsystem.turnCenter(speed, timevar, lOrRbool);
    }
}
