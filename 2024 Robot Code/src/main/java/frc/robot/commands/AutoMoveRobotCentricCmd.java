package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

import java.util.function.Supplier;

/*
 * This is a command that will be used to drive the robot with a joystick
 * This command originated from the CTRE Phoenix Swerve Drive Generated Code
 */
public class AutoMoveRobotCentricCmd extends Command {

    private final double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

    private final SwerveDriveSubsystem driveSubsystem;


    private final double xVelocityFeetPerSec;
    private final double yVelocityFeetPerSec;
    private final double rotationalRateDegPerSec;


    private final SwerveRequest.RobotCentric robotCentricDriveRequest = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.005).withRotationalDeadband(MaxAngularRate * 0.005)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);  
        
    
    
    
    public AutoMoveRobotCentricCmd(SwerveDriveSubsystem SwerveDriveSubsystem, double forwardFeetPerSec, double rightFeetPerSec, double rotationalRateDegPerSec) {
        
        driveSubsystem = SwerveDriveSubsystem;
        this.xVelocityFeetPerSec = forwardFeetPerSec;
        this.yVelocityFeetPerSec = -rightFeetPerSec;
        this.rotationalRateDegPerSec = rotationalRateDegPerSec;

        

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveDriveSubsystem);


    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // TODO: make sure these directions are correct

        driveSubsystem.setControl(
                robotCentricDriveRequest.withVelocityX(Units.feetToMeters(xVelocityFeetPerSec))
                .withVelocityY(Units.feetToMeters(yVelocityFeetPerSec))
                .withRotationalRate(Units.degreesToRadians(rotationalRateDegPerSec))
        );
        
    }

    

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}