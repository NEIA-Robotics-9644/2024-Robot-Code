package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhysicalRobotCharacteristics;
import frc.robot.subsystems.shooter.Motor;

public class ShooterSubsystem extends SubsystemBase{
    private final Motor[] flywheels = new Motor[2];
    private final Motor[] swivel = new Motor[2];
    private final Motor[] feeder = new Motor[1];

    public ShooterSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterSubsystem(MotorIO leftFlywheel, MotorIO rightFlywheel, MotorIO swivel1, MotorIO swivel2, MotorIO feeder) {

        if (leftFlywheel == null || rightFlywheel == null || swivel1 == null || swivel2 == null || feeder == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.flywheels[0] = new Motor(leftFlywheel);
        this.flywheels[1] = new Motor(rightFlywheel);
        this.swivel[0] = new Motor(swivel1);
        this.swivel[1] = new Motor(swivel2);
        this.feeder[0] = new Motor(feeder);
    }
    @Override
    public void periodic() {
        for (Motor motor : flywheels) {
            motor.periodic();
        }
        for (Motor motor : swivel) {
            motor.periodic();
        }
        feeder[0].periodic();
    }
    public void swivel()
    {

    }
    public void shoot(double voltage)
    {
        flywheels[0].setActive(voltage);
        flywheels[1].setActive(-voltage);
    }
    public void feeder()
    {

    }
}