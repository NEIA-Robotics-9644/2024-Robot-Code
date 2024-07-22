package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MoveShooterToManualAngleCmd;
import frc.robot.commands.ShootWhenReadyCmd;
import frc.robot.commands.SpinShooterWheelsCmd;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoCreator {

    private final ShuffleboardTab autoTab;
    

    private final GenericEntry shouldShoot;
    private final GenericEntry shouldDrive;

    private final GenericEntry startDelay;
    private final GenericEntry driveDelay;
    private final GenericEntry shooterWheelPercentSpeed;
    private final GenericEntry feederWheelPercentSpeed;
    private final GenericEntry shooterAngle;
    private final GenericEntry angleMoveDuration;
    private final GenericEntry shootNoteDuration;
    private final GenericEntry movementPathDuration;

    private final GenericEntry isRed;

    private final SendableChooser<Command> autoChooser;



    private final ShooterSubsystem shooter;
    private final SwerveDriveSubsystem drive;
    private final ClimberSubsystem climber;
    

    /*
     * Much of this code does not work
     * The auto that is created is tested, but there is a lot of stuff here that is almost integrated but not quite working yet.
     */
    public AutoCreator(ShooterSubsystem shooter, SwerveDriveSubsystem drive, ClimberSubsystem climber) {
        autoTab = Shuffleboard.getTab("Auto");
        shouldShoot = autoTab.add("Should Shoot", true).withWidget("Toggle Switch").withPosition(0,0).getEntry();
        shouldDrive = autoTab.add("Should Drive", true).withWidget("Toggle Switch").withPosition(1,0).getEntry();
        shooterWheelPercentSpeed = autoTab.add("Shooter Wheel Speed %", 1).withSize(2, 1).withPosition(6,0).getEntry();
        feederWheelPercentSpeed = autoTab.add("Feeder Wheel Speed %", 1).withSize(2, 1).withPosition(4,0).getEntry();

        
        // TODO: Add angles for side and front of speaker
        // ~50 is good for front, ~47 is good for side
        // But that's subject to change
        shooterAngle = autoTab.add("Shooter Angle", 46).withSize(1,1).withPosition(3,0).getEntry();
        startDelay = autoTab.add("Start Delay", 0).withSize(1,1).withPosition(0,2).getEntry();
        angleMoveDuration = autoTab.add("Angle Move Time", 4).withSize(2,1).withPosition(1,2).getEntry();
        shootNoteDuration = autoTab.add("Shoot Note Time", 1).withSize(2,1).withPosition(3,2).getEntry();
        driveDelay = autoTab.add("Drive Delay", 0).withSize(1,1).withPosition(5,2).getEntry();
        movementPathDuration = autoTab.add("Movement Path Time", 10).withSize(2,1).withPosition(6,2).getEntry();

        isRed = autoTab.add("Is Red", false).withSize(1, 1).withPosition(6, 1).getEntry();

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Move Forward", new PathPlannerAuto("DriveForward"));
        autoChooser.addOption("Origin Move Forward", new PathPlannerAuto("OriginDriveForward"));
        autoChooser.addOption("Move Forward From Speaker Left", new PathPlannerAuto("DriveForwardFromSpeakerLeft"));
        autoChooser.addOption("Origin Move Forward From Speaker Left", new PathPlannerAuto("OriginDriveForwardFromSpeakerLeft"));
        autoChooser.addOption("Move Forward From Speaker Right", new PathPlannerAuto("DriveForwardFromSpeakerRight"));
        autoChooser.addOption("Origin Move Forward From Speaker Right", new PathPlannerAuto("OriginDriveForwardFromSpeakerRight"));
        
    


        autoTab.add(autoChooser).withSize(2, 1).withPosition(0,1);

        this.shooter = shooter;
        this.drive = drive;
        this.climber = climber;

    }

    public Command createAuto() {
        boolean shouldShoot = this.shouldShoot.getBoolean(true);
        boolean shouldDrive = this.shouldDrive.getBoolean(true);

        double startDelay = this.startDelay.getDouble(0);
        double driveDelay = this.driveDelay.getDouble(0);
        double shooterWheelPercentSpeed = this.shooterWheelPercentSpeed.getDouble(1);
        double feederWheelPercentSpeed = this.feederWheelPercentSpeed.getDouble(0.4);      
        double shooterAngle = this.shooterAngle.getDouble(25);
        double angleMoveDuration = this.angleMoveDuration.getDouble(4);
        double shootNoteDuration = this.shootNoteDuration.getDouble(1);
        Command movementPath = new PathPlannerAuto(autoChooser.getSelected().getName());

        

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        /*
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
        */

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

        
        if (shouldDrive) {
            /*
            autoCommand.addCommands(
                    Commands.runOnce(drive::initializePathPlanner),
                    movementPath.withTimeout(1),
                    new MoveShooterToBottomAndResetCmd(shooter, 0.05).withTimeout(5)
                
            );
            */

            /*
            autoCommand.addCommands(
                new JoystickDriveCmd(drive, () -> -1.0, () -> -0.2, () -> 0.0, () -> false, () -> false, () -> true, () -> false).withTimeout(4)
            );
            */
        
        } else {
            /*
            autoCommand.addCommands(
                new MoveShooterToBottomAndResetCmd(shooter, 0.05).withTimeout(5)
            );
            */
        }

        

        return autoCommand;
    }

    
}
