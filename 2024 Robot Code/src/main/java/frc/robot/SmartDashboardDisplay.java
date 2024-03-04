package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SmartDashboardDisplay extends SubsystemBase {

    private final SwerveDriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;


    public SmartDashboardDisplay(SwerveDriveSubsystem swerveDriveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.drive = swerveDriveSubsystem;
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;
    }

    public void periodic() {
        


        SmartDashboard.putString("Shooter Wheels", Math.round(shooter.getShooterWheelsSpeedPercent() * 100.0) + "% Speed");
        SmartDashboard.putString("Shooter Angle", Math.round(shooter.getShooterAngleDeg()) + "°");
        SmartDashboard.putString("Shooter Setpoint", Math.round(shooter.getShooterAngleSetpointDeg()) + "°");


        SmartDashboard.putString("Intake Wheels", Math.round(intake.getIntakeWheelsPercentVelocity() * 100.0) + "% Speed");
        SmartDashboard.putString("Intake Extender", Math.round(intake.getExtendAngleDeg()) + "°");
        
    }
    
}
