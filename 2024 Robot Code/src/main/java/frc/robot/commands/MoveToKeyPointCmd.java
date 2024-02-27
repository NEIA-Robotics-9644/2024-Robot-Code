package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.Constants.PhysicalRobotCharacteristics;
import static frc.robot.Constants.*;


public class MoveToKeyPointCmd extends Command {

    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> omegaSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final Supplier<String> pointSupplier;
    private final DriveSubsystem driveSubsystem;


    public MoveToKeyPointCmd(
            DriveSubsystem driveSubsystem, Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> omegaSupplier, Supplier<Boolean> fieldOrientedSupplier, Supplier<String> pointSupplier) {

        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.pointSupplier = pointSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get values
        double x = xSupplier.get();
        double y = ySupplier.get();
        double omega = omegaSupplier.get();
        boolean fieldOriented = fieldOrientedSupplier.get();
        String point = pointSupplier.get();

        // Apply a deadband
        double deadband = 0.1;

        int index = 0;

        for (int i = 0; i < KeyPoints.keyPointsName.length; i++) {
            if (KeyPoints.keyPointsName[i].equals(point)) 
            {
                index = i;
                break;
            }
        }
        Pose2d keyPoint = KeyPoints.positions[index];

        x = -MathUtil.applyDeadband(keyPoint.getX() - driveSubsystem.WhateverIsTheVariableForPosition.getX(), deadband, 1);
        y = MathUtil.applyDeadband(keyPoint.getY() - driveSubsystem.WhateverIsTheVariableForPosition.getY(), deadband, 1);
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