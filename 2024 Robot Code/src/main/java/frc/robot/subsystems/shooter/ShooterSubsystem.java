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

    public static final double FEEDER_RUN_VOLTAGE = 12.0;

    private final Motor[] flywheels = new Motor[2];
    // private final Motor[] swivel = new Motor[2];
    private final Motor[] feeder = new Motor[1];

    public ShooterSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterSubsystem(MotorIO leftFlywheel, MotorIO rightFlywheel, MotorIO feeder) {

        if (leftFlywheel == null || rightFlywheel == null || feeder == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.flywheels[0] = new Motor(leftFlywheel);
        this.flywheels[1] = new Motor(rightFlywheel);
        // this.swivel[0] = new Motor(swivel1);
        // this.swivel[1] = new Motor(swivel2);
        this.feeder[0] = new Motor(feeder);
        this.feeder[0].setBrake(true);
    }
    @Override
    public void periodic() {
        for (Motor motor : flywheels) {
            motor.periodic();
        }
        //for (Motor motor : swivel) {
        //    motor.periodic();
        //}
        feeder[0].periodic();
    }
    public void swivel(double angle) {
        // swivel[0].setAngle(angle);
        // swivel[1].setAngle(angle);
    }

    /*
     * Spin the shooter wheels
     * The input should be from -1 to 1
     */
    public void shoot(double percent) {
        flywheels[0].setMotorSpeed(percent * flywheels[0].getMaxMotorVoltage());
        flywheels[1].setMotorSpeed(-percent * flywheels[1].getMaxMotorVoltage());
    }

    /*
     * Set whether to spin the feeder wheel
     */
    public void setFeeder(boolean running) {
        feeder[0].setMotorSpeed(running ? FEEDER_RUN_VOLTAGE : 0.0);
    }

    public void setFeederSpeed(double percent) {
        feeder[0].setMotorSpeed(percent * feeder[0].getMaxMotorVoltage());
    }
}