package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhysicalRobotCharacteristics;

public class DriveSubsystem extends SubsystemBase {
    
    private final Module[] modules = new Module[4]; // FL, FR, BR, BL
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(PhysicalRobotCharacteristics.moduleTranslations);
    private final Gyro gyro;

    // No empty constructor
    public DriveSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public DriveSubsystem(ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backRight, ModuleIO backLeft, GyroIO gyro) {
        // Check for null hardware
        if (frontLeft == null || frontRight == null || backRight == null || backLeft == null || gyro == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize modules
        this.modules[0] = new Module(frontLeft);
        this.modules[1] = new Module(frontRight);
        this.modules[2] = new Module(backRight);
        this.modules[3] = new Module(backLeft);

        this.gyro = new Gyro(gyro);

        new java.util.Timer().schedule( 
        new java.util.TimerTask() {
            @Override
            public void run() {
                gyro.resetHeading();
            }
        }, 
        1000
);
    }



    public double getAngleDeg() {
        return gyro.getAngleDeg();
    }

    @Override
    public void periodic() {
        for (Module module : modules) {
            module.periodic();
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SmartDashboard.putNumber("Heading", gyro.getAngleDeg());
        // Calculate module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Set module states
        for (int i = 0; i < 4; i++) {
            modules[i].drive(moduleStates[i]);
        }
    }    
}