package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final VisionIO visionIO;

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, VisionIO visionIO, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        if (visionIO == null) {
            throw new IllegalArgumentException("VisionIO cannot be null");
        }

        this.visionIO = visionIO;

        
    }
    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, VisionIO visionIO, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        if (visionIO == null) {
            throw new IllegalArgumentException("VisionIO cannot be null");
        }

        this.visionIO = visionIO;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


    @Override
    public void periodic() {
        var visionResult = visionIO.getEstimatedGlobalPose();


        if (visionResult.isPresent()) {

            // Make a pose2d from the pose3d
            var translation = visionResult.get().estimatedPose.getTranslation();
            var rotation = visionResult.get().estimatedPose.getRotation();
            var pose = new Pose2d(translation.getX(), translation.getY(), new Rotation2d(rotation.getZ()));

            addVisionMeasurement(pose, visionResult.get().timestampSeconds, visionIO.getEstimationStdDevs(pose));
        }


        SmartDashboard.putString("Pose", this.m_odometry.getEstimatedPosition().toString());

        SmartDashboard.putNumber("Gyro Angle (Degrees)", this.getPigeon2().getAngle());
    
    }
    
}
