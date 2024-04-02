package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SmartDashboardDisplay extends SubsystemBase {

    private final SwerveDriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;
    private final SendableChooser auto;


    public SmartDashboardDisplay(SwerveDriveSubsystem swerveDriveSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem, SendableChooser autoChooser) {
        this.drive = swerveDriveSubsystem;
        this.shooter = shooterSubsystem;
        this.climber = climberSubsystem;
        this.auto = autoChooser;
    }

    public void periodic() {
        


        SmartDashboard.putString("Shooter Wheels", Math.round(shooter.getShooterWheelsSpeedPercent() * 100.0) + "% Speed");
        SmartDashboard.putString("Shooter Angle", Math.round(shooter.getShooterAngleDeg() * 100) / 100.0 + "°");
        SmartDashboard.putString("Feeder Wheels", Math.round(shooter.getFeederVelocityPercent() * 100.0) + "% Velocity");

        SmartDashboard.putString("Shooter Setpoint", "#" + shooter.getSetpointIndex());
        SmartDashboard.putString("Shooter Angle Setpoint", shooter.getAngleSetpoint() + "°");
        SmartDashboard.putString("Shooter Wheel Setpoint", Math.round(shooter.getWheelSpeedSetpoint() * 100.0) + " % Speed");
        SmartDashboard.putString("Feeder Wheel Setpoint", Math.round(shooter.getFeederSpeedSetpoint() * 100.0) + " % Speed");

        SmartDashboard.putBoolean("Note Detected", shooter.noteDetected());
        
        SmartDashboard.putString("Climber Position", climber.getClimberRotations() + " Rotations");
        SmartDashboard.putString("Climber Speed", climber.getClimberSpeed() + " RPM");
        SmartDashboard.putData(auto);
    }
    
}
