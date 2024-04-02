package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveRobotCentricCmd;
import frc.robot.commands.JoystickDriveCmd;
import frc.robot.commands.MoveShooterToBottomAndResetCmd;
import frc.robot.commands.MoveShooterToManualAngleCmd;
import frc.robot.commands.ShootWhenReadyCmd;
import frc.robot.commands.SpinShooterWheelsCmd;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoCreator {

    private final ShuffleboardTab autoTab;
    

    private final GenericEntry shouldShoot;
    private final GenericEntry shouldPathPlannerDrive;
    private final GenericEntry shouldManualDrive;

    private final GenericEntry startDelay;
    private final GenericEntry driveDelay;
    private final GenericEntry shooterWheelPercentSpeed;
    private final GenericEntry feederWheelPercentSpeed;
    private final GenericEntry shooterAngle;
    private final GenericEntry angleMoveDuration;
    private final GenericEntry shootNoteDuration;

    // TODO: Check if this is correct
    // Forward is the "intake" side, oposite the side we can shoot
    // Right is the right side of the robot if the intake side is forward
    private final GenericEntry manualDriveForwardFeetPerSec;
    private final GenericEntry manualDriveRightFeetPerSec;
    private final GenericEntry manualDriveDuration;

    private final GenericEntry isRed;

    private final SendableChooser<Command> pathPlannerAutoChooser;



    private final ShooterSubsystem shooter;
    private final SwerveDriveSubsystem drive;
    private final ClimberSubsystem climber;
    
    public AutoCreator(ShooterSubsystem shooter, SwerveDriveSubsystem drive, ClimberSubsystem climber) {
        autoTab = Shuffleboard.getTab("Auto");
        shouldShoot = autoTab.add("Should Shoot", true).withWidget("Toggle Switch").getEntry();
        shouldPathPlannerDrive = autoTab.add("Should PathPlanner Drive", false).getEntry();
        shouldManualDrive = autoTab.add("Should Manual Drive", false).getEntry();
        shooterAngle = autoTab.add("Shooter Angle", 46).getEntry();
        
        feederWheelPercentSpeed = autoTab.add("Feeder Wheel Speed %", 1).getEntry();
        shooterWheelPercentSpeed = autoTab.add("Shooter Wheel Speed %", 1).getEntry();
        
        isRed = autoTab.add("Is Red", false).getEntry();


        
        manualDriveForwardFeetPerSec = autoTab.add("Manual Drive Forward Speed (FeetPerSec)", 0.5).getEntry();
        manualDriveRightFeetPerSec = autoTab.add("Manual Drive Right Speed (FeetPerSec)", 0.5).getEntry();
        manualDriveDuration = autoTab.add("Manual Drive Duration", 5).getEntry();

        

        startDelay = autoTab.add("Start Delay", 0).getEntry();
        angleMoveDuration = autoTab.add("Angle Move Time", 4).getEntry();
        shootNoteDuration = autoTab.add("Shoot Note Time", 1).getEntry();
        driveDelay = autoTab.add("Drive Delay", 0).getEntry();


        pathPlannerAutoChooser = new SendableChooser<Command>();


        pathPlannerAutoChooser.setDefaultOption("Move Forward", new PathPlannerAuto("DriveForward"));
        pathPlannerAutoChooser.addOption("Origin Move Forward", new PathPlannerAuto("OriginDriveForward"));
        pathPlannerAutoChooser.addOption("Move Forward From Speaker Left", new PathPlannerAuto("DriveForwardFromSpeakerLeft"));
        pathPlannerAutoChooser.addOption("Origin Move Forward From Speaker Left", new PathPlannerAuto("OriginDriveForwardFromSpeakerLeft"));
        pathPlannerAutoChooser.addOption("Move Forward From Speaker Right", new PathPlannerAuto("DriveForwardFromSpeakerRight"));
        pathPlannerAutoChooser.addOption("Origin Move Forward From Speaker Right", new PathPlannerAuto("OriginDriveForwardFromSpeakerRight"));
        

    


        autoTab.add(pathPlannerAutoChooser);

        this.shooter = shooter;
        this.drive = drive;
        this.climber = climber;

    }

    public Command createAuto() {
        boolean shouldShoot = this.shouldShoot.getBoolean(true);
        boolean shouldPathPlannerDrive = this.shouldPathPlannerDrive.getBoolean(false);
        boolean shouldManualDrive = this.shouldManualDrive.getBoolean(false);

        double startDelay = this.startDelay.getDouble(0);
        double driveDelay = this.driveDelay.getDouble(0);
        double shooterWheelPercentSpeed = this.shooterWheelPercentSpeed.getDouble(1);
        double feederWheelPercentSpeed = this.feederWheelPercentSpeed.getDouble(0.4);      
        double shooterAngle = this.shooterAngle.getDouble(25);
        double angleMoveDuration = this.angleMoveDuration.getDouble(4);
        double shootNoteDuration = this.shootNoteDuration.getDouble(1);
        Command movementPath = new PathPlannerAuto(pathPlannerAutoChooser.getSelected().getName());
        

        double manualDriveForwardFeetPerSec = this.manualDriveForwardFeetPerSec.getDouble(0.5);
        double manualDriveRightFeetPerSec = this.manualDriveRightFeetPerSec.getDouble(0.5);
        double manualDriveDuration = this.manualDriveDuration.getDouble(5);

        

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        autoCommand.addCommands(
            new Command () {
                @Override
                public void execute() {
                    drive.setFieldSide(isRed.getBoolean(false));
                    System.out.println("Field Side: " + isRed.getBoolean(false));
                    drive.resetPose(new Pose2d(0, 0, new Rotation2d(0.0)));
                }

                @Override
                public boolean isFinished() {
                    return true;
                }
            },
            new WaitCommand(startDelay)
        );

        if (shouldShoot) {
            autoCommand.addCommands(
                new ParallelDeadlineGroup(
                    new MoveShooterToManualAngleCmd(shooter, shooterAngle, shooterWheelPercentSpeed, feederWheelPercentSpeed).withTimeout(angleMoveDuration),
                    new SpinShooterWheelsCmd(shooter)
                ),
                new ShootWhenReadyCmd(shooter, 0.1, 0.99).withTimeout(shootNoteDuration)
            
            );
        }

        autoCommand.addCommands(
            new WaitCommand(driveDelay)
        );

        
        if (shouldPathPlannerDrive) {
            
            autoCommand.addCommands(
                    Commands.runOnce(drive::initializePathPlanner),
                    movementPath,
                    new MoveShooterToBottomAndResetCmd(shooter, 0.05).withTimeout(5)
                
            );
            

        
        }
        
        if (shouldManualDrive) {
            autoCommand.addCommands(
                new AutoMoveRobotCentricCmd(drive, manualDriveForwardFeetPerSec, manualDriveRightFeetPerSec, 0).withTimeout(manualDriveDuration)
            );
        }



        return autoCommand;
    }

    
}
