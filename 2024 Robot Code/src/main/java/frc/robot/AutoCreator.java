package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveRobotCentricCmd;
import frc.robot.commands.MoveShooterToBottomAndResetCmd;
import frc.robot.commands.MoveShooterToManualAngleCmd;
import frc.robot.commands.ShootWhenReadyCmd;
import frc.robot.commands.SpinShooterWheelsNoRumbleCmd;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoCreator {

    
    // Forward is the "intake" side, oposite the side we can shoot
    // Right is the right side of the robot if the intake side is forward
    

    private final ShooterSubsystem shooter;
    private final SwerveDriveSubsystem drive;
    private final ClimberSubsystem climber;
    
    public AutoCreator(ShooterSubsystem shooter, SwerveDriveSubsystem drive, ClimberSubsystem climber) {

        SmartDashboard.putString("Auto/Config", "Forward setpoint: 52 deg, 100% feeder, 100% shooter.  Side setpoint: 49 deg, 100% feeder, 100% shooter, rotate the robot");



        if (!SmartDashboard.containsKey("Auto/Should Shoot")) {
            SmartDashboard.putBoolean("Auto/Should Shoot", true);
        }

        if (!SmartDashboard.containsKey("Auto/Should Manual Drive")) {
            SmartDashboard.putBoolean("Auto/Should Manual Drive", false);
        }

        if (!SmartDashboard.containsKey("Auto/Feeder Wheel Speed %")) {
            SmartDashboard.putNumber("Auto/Feeder Wheel Speed %", 0.4);
        }

        if (!SmartDashboard.containsKey("Auto/Shooter Wheel Speed %")) {
            SmartDashboard.putNumber("Auto/Shooter Wheel Speed %", 1);
        }

        if (!SmartDashboard.containsKey("Auto/Shooter Angle")) {
            SmartDashboard.putNumber("Auto/Shooter Angle", 49);
        }

        if (!SmartDashboard.containsKey("Auto/Manual Drive Forward Speed (FeetPerSec)")) {
            SmartDashboard.putNumber("Auto/Manual Drive Forward Speed (FeetPerSec)", 0.5);
        }

        if (!SmartDashboard.containsKey("Auto/Manual Drive Right Speed (FeetPerSec)")) {
            SmartDashboard.putNumber("Auto/Manual Drive Right Speed (FeetPerSec)", 0.5);
        }

        if (!SmartDashboard.containsKey("Auto/Manual Drive CCW Rotation (DegPerSec)")) {
            SmartDashboard.putNumber("Auto/Manual Drive CCW Rotation (DegPerSec)", 0.5);
        }

        if (!SmartDashboard.containsKey("Auto/Manual Drive Duration")) {
            SmartDashboard.putNumber("Auto/Manual Drive Duration", 5);
        }



        if (!SmartDashboard.containsKey("Auto/Start Delay")) {
            SmartDashboard.putNumber("Auto/Start Delay", 0);
        }

        if (!SmartDashboard.containsKey("Auto/Angle Reset Time")) {
            SmartDashboard.putNumber("Auto/Angle Reset Time", 3);
        }

        if (!SmartDashboard.containsKey("Auto/Angle Move Time")) {
            SmartDashboard.putNumber("Auto/Angle Move Time", 4);
        }

        if (!SmartDashboard.containsKey("Auto/Shoot Note Time")) {
            SmartDashboard.putNumber("Auto/Shoot Note Time", 1);
        }

        if (!SmartDashboard.containsKey("Auto/Drive Delay")) {
            SmartDashboard.putNumber("Auto/Drive Delay", 0);
        }

        /*
        pathPlannerAutoChooser = new SendableChooser<Command>();


        pathPlannerAutoChooser.setDefaultOption("Move Forward", new PathPlannerAuto("DriveForward"));
        pathPlannerAutoChooser.addOption("Origin Move Forward", new PathPlannerAuto("OriginDriveForward"));
        pathPlannerAutoChooser.addOption("Move Forward From Speaker Left", new PathPlannerAuto("DriveForwardFromSpeakerLeft"));
        pathPlannerAutoChooser.addOption("Origin Move Forward From Speaker Left", new PathPlannerAuto("OriginDriveForwardFromSpeakerLeft"));
        pathPlannerAutoChooser.addOption("Move Forward From Speaker Right", new PathPlannerAuto("DriveForwardFromSpeakerRight"));
        pathPlannerAutoChooser.addOption("Origin Move Forward From Speaker Right", new PathPlannerAuto("OriginDriveForwardFromSpeakerRight"));
        */

    


        // autoTab.add(pathPlannerAutoChooser);

        this.shooter = shooter;
        this.drive = drive;
        this.climber = climber;

    }

    public Command createAuto() {
        
        boolean shouldShoot = SmartDashboard.getBoolean("Auto/Should Shoot", false);
        boolean shouldManualDrive = SmartDashboard.getBoolean("Auto/Should Manual Drive", false);

        double startDelay = SmartDashboard.getNumber("Auto/Start Delay", 0);
        double driveDelay = SmartDashboard.getNumber("Auto/Drive Delay", 0);
        double shooterWheelPercentSpeed = SmartDashboard.getNumber("Auto/Shooter Wheel Speed %", 1);
        double feederWheelPercentSpeed = SmartDashboard.getNumber("Auto/Feeder Wheel Speed %", 0.4);    
        double shooterAngle = SmartDashboard.getNumber("Auto/Shooter Angle", 49);
        double shooterAngleResetDuration = SmartDashboard.getNumber("Auto/Angle Reset Time", 3);
        double angleMoveDuration = SmartDashboard.getNumber("Auto/Angle Move Time", 4);
        double shootNoteDuration = SmartDashboard.getNumber("Auto/Shoot Note Time", 1);
        //Command movementPath = new PathPlannerAuto(pathPlannerAutoChooser.getSelected().getName());
        

        double manualDriveForwardFeetPerSec = SmartDashboard.getNumber("Auto/Manual Drive Forward Speed (FeetPerSec)", 0.5);
        double manualDriveRightFeetPerSec = SmartDashboard.getNumber("Auto/Manual Drive Right Speed (FeetPerSec)", 0.5);
        double manualDriveCCWRotationDegPerSec = SmartDashboard.getNumber("Auto/Manual Drive CCW Rotation (DegPerSec)", 0);
        double manualDriveDuration = SmartDashboard.getNumber("Auto/Manual Drive Duration", 5);
        //boolean useVisionInAuto = this.useVisionInAuto.getBoolean(false);
        //boolean useVisionInTeleop = this.useVisionInTeleop.getBoolean(false);

        

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        autoCommand.addCommands(
            new Command () {
                @Override
                public void execute() {
                    //drive.useDataFromVision(useVisionInAuto);
                    //drive.setFieldSide(isRed.getBoolean(false));
                    //System.out.println("Field Side: " + isRed.getBoolean(false));
                    //drive.resetPose(new Pose2d(0, 0, new Rotation2d(0.0)));
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
                new MoveShooterToBottomAndResetCmd(shooter, 0.1).withTimeout(shooterAngleResetDuration),
                new ParallelCommandGroup(
                    new ParallelDeadlineGroup(
                        new MoveShooterToManualAngleCmd(shooter, shooterAngle, shooterWheelPercentSpeed, feederWheelPercentSpeed).withTimeout(angleMoveDuration + shootNoteDuration),
                        new SpinShooterWheelsNoRumbleCmd(shooter)
                    ),

                    new SequentialCommandGroup(
                        new WaitCommand(angleMoveDuration),
                        new ShootWhenReadyCmd(shooter, 0.1, 0.99).withTimeout(shootNoteDuration)
                    )
                )
            );
        }

        autoCommand.addCommands(
            new WaitCommand(driveDelay)
        );

        /*
        if (shouldPathPlannerDrive) {
            
            autoCommand.addCommands(
                    Commands.runOnce(drive::initializePathPlanner),
                    movementPath,
                    new MoveShooterToBottomAndResetCmd(shooter, 0.05).withTimeout(5)
                
            );
            

        
        }
        */
        
        if (shouldManualDrive) {
            autoCommand.addCommands(
                new AutoMoveRobotCentricCmd(drive, manualDriveForwardFeetPerSec, manualDriveRightFeetPerSec, manualDriveCCWRotationDegPerSec).withTimeout(manualDriveDuration)
            );
        }



        return autoCommand;
    }

    
}
