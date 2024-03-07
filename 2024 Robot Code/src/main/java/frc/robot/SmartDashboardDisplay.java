package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SmartDashboardDisplay extends SubsystemBase {

    private final SwerveDriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;


    public SmartDashboardDisplay(SwerveDriveSubsystem swerveDriveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem) {
        this.drive = swerveDriveSubsystem;
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;
        this.climber = climberSubsystem;
    }

    public void periodic() {
        


        SmartDashboard.putString("Shooter Wheels", Math.round(shooter.getShooterWheelsSpeedPercent() * 100.0) + "% Speed");
        SmartDashboard.putString("Shooter Angle", shooter.getShooterAngleDeg() + "°");
        SmartDashboard.putString("Shooter Setpoint", shooter.getShooterAngleSetpointDeg() + "°");
        SmartDashboard.putString("Feeder Wheels", Math.round(shooter.getFeederVelocityPercent() * 100.0) + "% Velocity");

        SmartDashboard.putBoolean("Note Detected", shooter.noteDetected());


        SmartDashboard.putString("Climber Position", climber.getClimberRotations() + " Rotations");
        SmartDashboard.putString("Climber Speed", climber.getClimberSpeed() + " RPM");

        // SmartDashboard.putString("Intake Wheels", Math.round(intake.getIntakeWheelsPercentVelocity() * 100.0) + "% Velocity");
        // SmartDashboard.putString("Intake Extender", Math.round(intake.getExtendAngleDeg()) + "°");


        
    }
    
}
