package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Teleoperated swerve drive command using inputs from two joysticks (controlling linear and angular velocities).
 */
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

        
        // Get chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                y * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                omega * driveSubsystem.getMaxAngularSpeedRadPerSec(),
                driveSubsystem.getRotation());
        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                y * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                omega * driveSubsystem.getMaxAngularSpeedRadPerSec(),
                Rotation2d.fromDegrees(0));
        }

        driveSubsystem.runVelocity(chassisSpeeds);
    }


    
}
