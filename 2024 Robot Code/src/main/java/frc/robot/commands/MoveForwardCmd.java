package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.Constants.PhysicalRobotCharacteristics;



public class MoveForwardCmd extends Command {

    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> directionForwardSupplier; // 1 is forward, -1 is backward
    private final Supplier<Double> directionSidewaysSupplier; // 1 is forward, -1 is backward
    private final DriveSubsystem driveSubsystem;


    public MoveForwardCmd(
            DriveSubsystem driveSubsystem, Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> directionForwardSupplier, Supplier<Double> directionSidewaysSupplier) {

        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.directionForwardSupplier = directionForwardSupplier;
        this.directionSidewaysSupplier = directionSidewaysSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get values
        double x = xSupplier.get();
        double y = ySupplier.get();
        double directionForward = directionForwardSupplier.get();
        double directionSideways = directionSidewaysSupplier.get();

        // Apply a deadband
        double deadband = 0.1;

        x = -MathUtil.applyDeadband(x, deadband, 1) * directionForward;
        y = -MathUtil.applyDeadband(y, deadband, 1) * directionSideways;

        // Get chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(x * PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec, y * PhysicalRobotCharacteristics.kMaxLinearSpeedMetersPerSec, 0);

        driveSubsystem.drive(chassisSpeeds);
    }
}