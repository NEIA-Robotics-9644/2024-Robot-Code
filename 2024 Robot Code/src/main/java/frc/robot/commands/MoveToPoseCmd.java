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
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Supplier<Boolean> sideSupplier;
    private final Supplier<Integer> indexSupplier;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final PIDController xPID = new PIDController(4, 0.001, 0.0);
    private final PIDController yPID = new PIDController(4, 0.001, 0.0);
    private final PIDController rotationPID = new PIDController(4, 0.001, 0.0);


    public MoveToPoseCmd(
            SwerveDriveSubsystem driveSubsystem,
            Supplier<Double> x, Supplier<Double> y, Supplier<Double> speed, Supplier<Double> rotation,
            Supplier<Boolean> red, Supplier<Integer> index) {

        this.swerveDriveSubsystem = driveSubsystem;
        this.xCord = x;
        this.yCord = y;
        this.rotationSupplier = rotation;
        this.speedPercentage = speed;
        this.sideSupplier = red;
        this.indexSupplier = index;

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

        x = Math.min(x, speedPercentage.get());
        y = Math.min(y, speedPercentage.get());
        rotation = Math.min(rotation, speedPercentage.get());


        // Apply request
        swerveDriveSubsystem.setControl(fieldCentric.withVelocityX(x)
            .withVelocityY(y)
            .withRotationalRate(rotation)
        );   
    }
    @Override
    public boolean isFinished() {
        if((Math.abs(swerveDriveSubsystem.getState().Pose.getX() - xCord.get()) < 0.05)
         && (Math.abs(swerveDriveSubsystem.getState().Pose.getY() - yCord.get()) < 0.05)
         && (Math.abs(swerveDriveSubsystem.getState().Pose.getRotation().getDegrees() - rotationSupplier.get()) < 2))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}           