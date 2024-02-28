package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

import java.util.function.Supplier;

/*
 * This is a command that will be used to drive the robot with a joystick
 */
public class JoystickDriveCmd extends Command {

    private final double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

    private final SwerveDriveSubsystem driveSubsystem;


    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> sidwaysSupplier;
    private final Supplier<Double> rotationalSupplier;


    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
    
    public JoystickDriveCmd(SwerveDriveSubsystem SwerveDriveSubsystem, Supplier<Double> forward, Supplier<Double> sideways, Supplier<Double> rotation) {
        
        driveSubsystem = SwerveDriveSubsystem;
        forwardSupplier = forward;
        sidwaysSupplier = sideways;
        rotationalSupplier = rotation;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveDriveSubsystem);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set the swerve drive to field relative
        
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
        return false;
    }
}
