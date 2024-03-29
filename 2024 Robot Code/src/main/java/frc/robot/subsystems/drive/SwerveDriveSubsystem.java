package frc.robot.subsystems.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

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

        initializePathPlanner();

        
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

        initializePathPlanner();
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

    public void initializePathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    Constants.PhysicalRobotCharacteristics.kDriveBaseRadiusMeters, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public Pose2d getPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    public Consumer<Pose2d> resetPose(Pose2d pose2d) {
        return pose -> this.m_odometry.resetPosition(new Rotation2d(0.0), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        pose2d);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
    
        return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
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
