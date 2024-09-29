package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Dashboard extends SubsystemBase {

    private final SwerveDriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;



    public Dashboard(SwerveDriveSubsystem swerveDrive, ShooterSubsystem shooter, ClimberSubsystem climber) {    
        this.drive = swerveDrive;
        this.shooter = shooter;
        this.climber = climber;


        SmartDashboard.putData("Tuning/Swerve Steer Gains", new GainsSendable(drive.steerConfig));
        
        SmartDashboard.putData("Tuning/Swerve Drive Gains", new GainsSendable(drive.driveConfig));
    }


    public class GainsSendable implements Sendable {
        
        private Slot0Configs config;

        public GainsSendable(Slot0Configs config) {
            this.config = config;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("kP", () -> config.kP, (double x) -> config.withKP(x));
            builder.addDoubleProperty("kI", () -> config.kI, (double x) -> config.withKI(x));
            builder.addDoubleProperty("kD", () -> config.kD, (double x) -> config.withKD(x));
            builder.addDoubleProperty("kS", () -> config.kS, (double x) -> config.withKS(x));
            builder.addDoubleProperty("kV", () -> config.kV, (double x) -> config.withKV(x));
            builder.addDoubleProperty("kA", () -> config.kA, (double x) -> config.withKA(x));
        }

    }

    public void periodic() {

        SmartDashboard.putNumber("Drive/Motor 0 Velocity", drive.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Motor 1 Velocity", drive.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Motor 2 Velocity", drive.getModule(2).getDriveMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Motor 3 Velocity", drive.getModule(3).getDriveMotor().getVelocity().getValueAsDouble());
        
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
