package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhysicalRobotCharacteristics;

public class IntakeSubsystem extends SubsystemBase{

    public IntakeSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public IntakeSubsystem(String test) {
        // Initialize things

        new java.util.Timer().schedule( 
        new java.util.TimerTask() {
            @Override
            public void run() {
                
            }
        }, 1000);
    }
    @Override
    public void periodic() {
        
    }
    public void intake()
    {

    }
    public void deploy()
    {
        
    }
}
