package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SmartDashboardDisplay extends SubsystemBase {

    private final SwerveDriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;



    public SmartDashboardDisplay(SwerveDriveSubsystem swerveDrive, ShooterSubsystem shooter, ClimberSubsystem climber) {    
        this.drive = swerveDrive;
        this.shooter = shooter;
        this.climber = climber;
    }

    public void periodic() {
        


        SmartDashboard.putString("Data/Shooter Wheels", Math.round(shooter.getShooterWheelsSpeedPercent() * 100.0) + "% Speed");
        SmartDashboard.putString("Data/Shooter Angle", Math.round(shooter.getShooterAngleDeg() * 100) / 100.0 + " deg");
        SmartDashboard.putString("Data/Feeder Wheels", Math.round(shooter.getFeederVelocityPercent() * 100.0) + "% Velocity");

        SmartDashboard.putString("Data/Shooter Setpoint", "#" + shooter.getSetpointIndex());
        SmartDashboard.putString("Data/Shooter Angle Setpoint", shooter.getAngleSetpoint() + " deg");
        SmartDashboard.putString("Data/Shooter Wheel Setpoint", Math.round(shooter.getWheelSpeedSetpoint() * 100.0) + " % Speed");
        SmartDashboard.putString("Data/Feeder Wheel Setpoint", Math.round(shooter.getFeederSpeedSetpoint() * 100.0) + " % Speed");

        SmartDashboard.putBoolean("Data/Note Detected", shooter.noteDetected());

        String pose = " X: " + Units.metersToInches(drive.getPose().getTranslation().getX()) + " ,";
        pose += " Y: " + Units.metersToInches(drive.getPose().getTranslation().getY()) + " ,";
        pose += " Rot: " + Math.round(drive.getPose().getRotation().getDegrees() * 100) / 100.0 + " deg";

        SmartDashboard.putString("Data/Pose", pose);

        SmartDashboard.putNumber("Data/Gyro Angle (Degrees)", drive.getPigeon2().getAngle());

        
    }
    
}
