// DOCUMENTATION UPDATED: 1/2/2024
// https://www.notion.so/neiafrc/SwerveJoystickCmd-9428899326394c69bc40cf2767d8e3d4?pvs=4

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command to drive the robot with joystick input.
 * This command should be run for the entirety of Teleop.
 */
public class SwerveJoystickCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /**
     * Creates a new SwerveJoystickCmd.
     * 
     * @param swerveSubsystem       The subsystem used by this command to drive.
     * @param xSpdFunction          A supplier whose output is the desired x speed
     *                              of the robot.
     * @param ySpdFunction          A supplier whose output is the desired y speed
     *                              of the robot.
     * @param turningSpdFunction    A supplier whose output is the desired turning
     *                              speed of the robot.
     * @param fieldOrientedFunction A supplier whose output is whether the robot
     *                              should drive in field-oriented mode.
     */
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        this.turningLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        addRequirements(swerveSubsystem);

    }

    @Override
    public void execute() {
        // 1. Get the joystick values
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();

        // Invert turning by default
        double turningSpeed = -turningSpdFunction.get();

        // 2. Apply a deadband to the joystick values
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

        // 3. Apply a slew rate limiter to the joystick values
        xSpeed = xLimiter.calculate(xSpeed) * SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSec;
        ySpeed = yLimiter.calculate(ySpeed) * SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSec;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSec;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            System.out.println("Field Oriented");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                    swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Calculate the desired module states
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Set the desired speeds on the drive motors
        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
