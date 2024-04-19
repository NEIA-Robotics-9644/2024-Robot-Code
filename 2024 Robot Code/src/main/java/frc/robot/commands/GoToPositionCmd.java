package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

public class GoToPositionCmd extends Command {

    private final double maxSpeed = 2; // Meters per second
    private final double maxAngularRate = 180; // Degrees per second

    private final SwerveDriveSubsystem drive;
    private final Supplier<Double> dx;
    private final Supplier<Double> dy;
    private final Supplier<Double> dTheta;

    private double targetX, targetY, targetTheta;

    private final PIDController xFeedback = new PIDController(4, 0.0, 0.0);
    private final PIDController yFeedback = new PIDController(4, 0.0, 0.0);
    private final PIDController thetaFeedback = new PIDController(4, 0.0, 0.0);

    private final RobotCentric request = new SwerveRequest.RobotCentric()
        .withDeadband(0.05)
        .withRotationalDeadband(1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public GoToPositionCmd(SwerveDriveSubsystem drive, Supplier<Double> dx, Supplier<Double> dy, Supplier<Double> dTheta) {


        this.drive = drive;
        this.dx = dx;
        this.dy = dy;
        this.dTheta = dTheta;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.targetX = drive.getPose().getTranslation().getX();
        this.targetY = drive.getPose().getTranslation().getY();
        this.targetTheta = drive.getPose().getRotation().getDegrees();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        targetX += dx.get();
        targetY += dy.get();
        targetTheta += dTheta.get();

        xFeedback.setSetpoint(targetX);
        yFeedback.setSetpoint(targetY);
        thetaFeedback.setSetpoint(targetTheta);

        double x = xFeedback.calculate(drive.getPose().getTranslation().getX());
        double y = yFeedback.calculate(drive.getPose().getTranslation().getY());
        double theta = thetaFeedback.calculate(drive.getPose().getRotation().getDegrees());

        // Cap speeds
        if (x > maxSpeed) x = maxSpeed;
        if (x < -maxSpeed) x = -maxSpeed;
        if (y > maxSpeed) y = maxSpeed;
        if (y < -maxSpeed) y = -maxSpeed;
        if (theta > maxAngularRate) theta = maxAngularRate;
        if (theta < -maxAngularRate) theta = -maxAngularRate;


        drive.setControl(
            request
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(theta)
        );

        String targetPose = "X: " + Units.metersToInches(targetX);
        targetPose += ", Y: " + Units.metersToInches(targetY);
        targetPose += ", Theta: " + targetTheta;
        SmartDashboard.putString("Data/Target Pose", targetPose);


    }
}
