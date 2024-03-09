package frc.robot.commands;


import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.Constants.PhysicalRobotCharacteristics;
import static frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;


public class MoveToPoseCmd extends Command {

    private final Supplier<Double> xCord;
    private final Supplier<Double> yCord;
    private final Supplier<Double> rotationSupplier;
    private final Supplier<Double> speedPercentage;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final PIDController xPID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController yPID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController rotationPID = new PIDController(0.1, 0.0, 0.0);


    public MoveToPoseCmd(
            SwerveDriveSubsystem driveSubsystem,
            Supplier<Double> x, Supplier<Double> y, Supplier<Double> speed, Supplier<Double> rotation, 
            Supplier<Boolean> fieldOrientedSupplier) {

        this.swerveDriveSubsystem = driveSubsystem;
        this.xCord = x;
        this.yCord = y;
        this.rotationSupplier = rotation;
        this.speedPercentage = speed;
        this.fieldOrientedSupplier = fieldOrientedSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        xPID.setSetpoint(xCord.get());
        yPID.setSetpoint(yCord.get());
        rotationPID.setSetpoint(rotationSupplier.get());
        double x = xPID.calculate(swerveDriveSubsystem.getState().Pose.getX(), xCord.get());
        double y = yPID.calculate(swerveDriveSubsystem.getState().Pose.getY(), yCord.get());
        double rotation = rotationPID.calculate(swerveDriveSubsystem.getState().Pose.getRotation().getDegrees(), rotationSupplier.get());

        // Apply request
        swerveDriveSubsystem.setControl(fieldCentric.withVelocityX(x * speedPercentage.get())
            .withVelocityY(y * speedPercentage.get())
            .withRotationalRate(rotation * speedPercentage.get())
        );   
    }
    @Override
    public boolean isFinished() {
        if(swerveDriveSubsystem.getState().Pose.getX() == xCord.get()
         && swerveDriveSubsystem.getState().Pose.getY() == yCord.get()
         && swerveDriveSubsystem.getState().Pose.getRotation().getDegrees() == rotationSupplier.get())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}           