package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.Constants.PhysicalRobotCharacteristics;



public class SwerveDriveCmd extends Command {

    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> omegaSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final DriveSubsystem driveSubsystem;


    public SwerveDriveCmd(
            DriveSubsystem driveSubsystem, Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> omegaSupplier, Supplier<Boolean> fieldOrientedSupplier) {

        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get values
        double x = xSupplier.get();
        double y = ySupplier.get();
        double omega = omegaSupplier.get();
        boolean fieldOriented = fieldOrientedSupplier.get();

        // Apply a deadband
        double deadband = 0.1;

        x = -MathUtil.applyDeadband(x, deadband, 1);
        y = MathUtil.applyDeadband(y, deadband, 1);
        omega = -MathUtil.applyDeadband(omega, deadband, 1);

        // Get chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec,
                y * PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec,
                omega * PhysicalRobotCharacteristics.kMaxAngularSpeedRadPerSec,
                new Rotation2d(-Math.toRadians(driveSubsystem.getAngleDeg())));
        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec,
                y * PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec,
                omega * PhysicalRobotCharacteristics.kMaxAngularSpeedRadPerSec,
                Rotation2d.fromDegrees(0));
        }

        driveSubsystem.drive(chassisSpeeds);
    }
}