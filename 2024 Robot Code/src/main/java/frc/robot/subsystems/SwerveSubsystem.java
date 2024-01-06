// NEEDS DOCUMENTING - 1/2/2024

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Simulation;
import frc.robot.Constants.SwerveDriveConstants;

import java.util.Set;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a swerve drive style drivetrain.
 * Uses {@link SwerveModule} objects to control the speed and angle of each
 * wheel.
 */
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            "Front Left",
            SwerveDriveConstants.kFrontLeftDriveMotorPort,
            SwerveDriveConstants.kFrontLeftTurningMotorPort,
            SwerveDriveConstants.kFrontLeftDriveEncoderReversed,
            SwerveDriveConstants.kFrontLeftTurningEncoderReversed,
            SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            "Front Right",
            SwerveDriveConstants.kFrontRightDriveMotorPort,
            SwerveDriveConstants.kFrontRightTurningMotorPort,
            SwerveDriveConstants.kFrontRightDriveEncoderReversed,
            SwerveDriveConstants.kFrontRightTurningEncoderReversed,
            SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            "Back Left",
            SwerveDriveConstants.kBackLeftDriveMotorPort,
            SwerveDriveConstants.kBackLeftTurningMotorPort,
            SwerveDriveConstants.kBackLeftDriveEncoderReversed,
            SwerveDriveConstants.kBackLeftTurningEncoderReversed,
            SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            "Back Right",
            SwerveDriveConstants.kBackRightDriveMotorPort,
            SwerveDriveConstants.kBackRightTurningMotorPort,
            SwerveDriveConstants.kBackRightDriveEncoderReversed,
            SwerveDriveConstants.kBackRightTurningEncoderReversed,
            SwerveDriveConstants.kBackRightDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(SwerveDriveConstants.kPigeonIMUCanId);

    private int timer;

    /**
     * Constructs a SwerveSubsystem.
     * Resets the gyro after a delay time to allow the gyro to calibrate.
     */
    public SwerveSubsystem() {
        // Put the call to reset on another thread and wait for 1 sec to let the gyro
        // recalibrate
        timer = 0;
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    /**
     * Resets the gyro so that it reads 0 degrees.
     * This can be used to reset the gyro during a match if there is drift in the
     * gyro,
     * or to zero the gyro during autonomous, or to zero the gyro before a match.
     * Returns {@link Set} of {@link Subsystem} objects so that it can be used as an
     * instant command.
     */
    public Set<Subsystem> zeroHeading() {
        gyro.reset();
        return null;
    }

    /**
     * Gets the current heading of the robot in degrees, via the gyro.
     * 
     * @return heading of the robot in degrees
     */
    public double getHeading() {
        return Simulation.simulationCompatable(
                Math.IEEEremainder(gyro.getAngle(), 360.0), "Gyro Angle", 0.0);
    }

    /**
     * Gets the current heading of the robot in a {@link Rotation2d} object, via the
     * gyro.
     * 
     * @return heading of the robot in a {@link Rotation2d} object
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Periodic method that is called by the scheduler.
     * Updates the gyro angle and the absolute encoder positions of the swerve
     * modules in SmartDashboard.
     */
    @Override
    public void periodic() {
        if (timer >= 10) {
            SmartDashboard.putNumber("Robot Heading", getHeading());
            SmartDashboard.putNumber("Front Left Abs Encoder", frontLeft.getTurningPosition());
            SmartDashboard.putNumber("Front Right Abs Encoder", frontRight.getTurningPosition());
            SmartDashboard.putNumber("Back Left Abs Encoder", backLeft.getTurningPosition());
            SmartDashboard.putNumber("Back Right Abs Encoder", backRight.getTurningPosition());
            timer = 0;
        } else {
            timer++;
        }
    }

    /**
     * Stops all of the swerve modules.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets the correct swerve modules to the states contained in the parameter
     * array.
     * 
     * @param desiredStates array of {@link SwerveModuleState} objects that contain
     *                      the desired states of the swerve modules
     *                      listed in the order: front left, front right, back left,
     *                      back right
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
};
