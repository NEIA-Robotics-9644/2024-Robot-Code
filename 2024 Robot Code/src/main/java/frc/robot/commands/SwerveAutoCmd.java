package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

import java.util.function.Supplier;

public class SwerveAutoCmd extends Command {

    private final double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

    private final SwerveDriveSubsystem driveSubsystem;


    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> sidwaysSupplier;
    private final Supplier<Double> rotationalSupplier;
    long startTime = System.nanoTime(); 
    private final Supplier<Double> targetTime; 
    double elapsedTime = 0;


    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
    
    public SwerveAutoCmd(SwerveDriveSubsystem SwerveDriveSubsystem, Supplier<Double> forward, Supplier<Double> sideways, Supplier<Double> rotation, Supplier<Double> time) {
        
        driveSubsystem = SwerveDriveSubsystem;
        forwardSupplier = forward;
        sidwaysSupplier = sideways;
        rotationalSupplier = rotation;
        targetTime = time;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveDriveSubsystem);


    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get the values
        double forward = forwardSupplier.get();
        double sideways = sidwaysSupplier.get();
        double rotation = rotationalSupplier.get();

        // Adapt the values to the robot
        double forwardOutput = -Math.abs(forward) * forward * MaxSpeed;
        double sidewaysOutput = -Math.abs(sideways) * sideways * MaxSpeed;
        double rotationOutput = -Math.abs(rotation) * rotation * MaxAngularRate;
        
        driveSubsystem.setControl(
                driveRequest.withVelocityX(forwardOutput)
                .withVelocityY(sidewaysOutput)
                .withRotationalRate(rotationOutput)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Set the swerve drive to field relative
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double time = targetTime.get();
        boolean runLoop = true;

        while(runLoop) {
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - startTime) / 1e9; // Convert nanoseconds to seconds
            elapsedTime = deltaTime;

            if(elapsedTime >= time)
            {
                runLoop = false;
            }
        }
        if(runLoop == false)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
