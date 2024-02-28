package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhysicalRobotCharacteristics;

public class IntakeSubsystem extends SubsystemBase{
    private final Motor[] deploy = new Motor[1];
    private final Motor[] feeder = new Motor[1];

    public IntakeSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public IntakeSubsystem(MotorIO deploy, MotorIO feeder) {

        if (deploy == null || feeder == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.deploy[0] = new Motor(deploy);
        this.feeder[0] = new Motor(feeder);
    }
    @Override
    public void periodic() {
        deploy[0].periodic();
        feeder[0].periodic();
    }
    public void deploy(double voltage)
    {
        deploy[0].setActive(voltage);
    }
    public void unDeploy(double voltage)
    {
        deploy[0].setActive(-voltage);
    }
    public void feeder(double voltage)
    {
        feeder[0].setActive(voltage);
    }
}
