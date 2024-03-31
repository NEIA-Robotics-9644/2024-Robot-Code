// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.Utils;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Modes;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.JoystickDriveCmd;
import frc.robot.commands.MoveHookCmd;
import frc.robot.commands.MoveShooterToBottomAndResetCmd;
import frc.robot.commands.RunSourceIntakeCmd;
import frc.robot.commands.ShootWhenReadyCmd;
import frc.robot.commands.SpinShooterWheelsCmd;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.climberMotor.ClimberMotorIOSim;
import frc.robot.subsystems.climber.climberMotor.ClimberMotorIOSparkMax;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.shooter.feederWheel.FeederWheelIOSim;
import frc.robot.subsystems.shooter.feederWheel.FeederWheelIOSparkMax;
import frc.robot.subsystems.shooter.shooterAngle.ShooterAngleIOSparkMax;
import frc.robot.subsystems.shooter.shooterWheel.ShooterWheelIOTalonFX;
import frc.robot.subsystems.shooter.shooterWheel.ShooterWheelIOSim;
import frc.robot.subsystems.shooter.shooterAngle.ShooterAngleIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.noteSensor.NoteSensorIORoboRio;
import frc.robot.subsystems.shooter.noteSensor.NoteSensorIOSim;
import frc.robot.subsystems.hook.HookSubsystem;
import frc.robot.subsystems.hook.hookMotor.HookMotorIOSim;
import frc.robot.subsystems.hook.hookMotor.HookMotorIOSparkMax;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Modes mode;
  
  
  // private double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  // private double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);


  private final SwerveDriveSubsystem drivetrain = DriveConstants.DriveTrain; // My drivetrain
  
  private final ShooterSubsystem shooter;
  
  
  private final ClimberSubsystem climber;
 
  private final HookSubsystem hook;
  
  
  private final SmartDashboardDisplay display;


  AutoCreator autoCreator;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Utils.isSimulation()) {
      mode = Modes.SIM;
    } else {
      mode = Modes.REAL;
    }

    // Constants.DriveConstants.putValues();

    if (mode == Modes.REAL) {
      shooter = new ShooterSubsystem(
          new ShooterWheelIOTalonFX(21),
          new ShooterWheelIOTalonFX(22),
          new FeederWheelIOSparkMax(23),
          new ShooterAngleIOSparkMax(24, 25),
          new NoteSensorIORoboRio(),
          new double[] { 0, 25, 55, 70},
          new double[] { 1, 0.4, 1, 0.4},
          new double[] { 1, 0.4, 1, 0.4}
      );

      climber = new ClimberSubsystem(
          new ClimberMotorIOSparkMax(26),
          new ClimberMotorIOSparkMax(27)
      ); 

      hook = new HookSubsystem(new HookMotorIOSparkMax(28));

    } else {
      shooter = new ShooterSubsystem(
          new ShooterWheelIOSim(),
          new ShooterWheelIOSim(),
          new FeederWheelIOSim(),
          new ShooterAngleIOSim(),
          new NoteSensorIOSim(),
          new double[] { 0, 25, 52, 70},
          new double[] { 1, 0.4, 1, 0.4},
          new double[] { 1, 0.4, 1, 0.4}
      );

      climber = new ClimberSubsystem(
          new ClimberMotorIOSim(),
          new ClimberMotorIOSim()
      );

      hook = new HookSubsystem(new HookMotorIOSim());
    }
    


    display = new SmartDashboardDisplay(drivetrain, shooter, climber, hook);



    // Configure the trigger bindings
    configureBindings();


    autoCreator = new AutoCreator(shooter, drivetrain, climber);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    var operatorHID = operatorController.getHID();
    var driverHID = driverController.getHID();
    
    drivetrain.setDefaultCommand(new JoystickDriveCmd(drivetrain, driverHID::getLeftY, driverHID::getLeftX, 
        driverHID::getRightX, driverHID::getRightBumper, driverHID::getLeftBumper, 
        () -> !(driverHID.getLeftTriggerAxis() > 0.5), () -> (driverHID.getPOV() == 270)));
    
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }


    

    // SHOOTER COMMANDS

    var oLeftTrigger = new Trigger(() -> operatorHID.getLeftTriggerAxis() > 0.5);
    oLeftTrigger.whileTrue(new SpinShooterWheelsCmd(shooter));

    
    var oLeftBumper = new Trigger(() -> operatorHID.getLeftBumper());
    oLeftBumper.whileTrue(new ShootWhenReadyCmd(shooter, 0.9, 0.8));

    // Bottom
    var oATrigger = new Trigger(() -> operatorHID.getAButton());
    oATrigger.whileTrue(Commands.run(() -> shooter.goToSetpoint(0)));
    
    // Protected Shot
    var oBTrigger = new Trigger(() -> operatorHID.getBButton());
    oBTrigger.whileTrue(Commands.run(() -> shooter.goToSetpoint(1)));
    
    // Source Intake
    var oXTrigger = new Trigger(() -> operatorHID.getXButton());
    oXTrigger.whileTrue(Commands.run(() -> shooter.goToSetpoint(2)));

    // Speaker
    var oYTrigger = new Trigger(() -> operatorHID.getYButton());
    //oYTrigger.onTrue(Commands.runOnce(() -> shooter.goToSetpoint(3)));
    
    

    // Reset Shooter Angle
    var oStartTrigger = new Trigger(() -> operatorHID.getStartButton());
    oStartTrigger.whileTrue(new MoveShooterToBottomAndResetCmd(shooter, 0.05));


    // Adjust the shooter angle of this setpoint
    var oPOVUpTrigger = new Trigger(() -> operatorHID.getPOV() == 0);
    oPOVUpTrigger.onTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(1)));
    
    var oPOVDownTrigger = new Trigger(() -> operatorHID.getPOV() == 180);
    oPOVDownTrigger.onTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(-1)));
    


    // Adjust the shooter wheel speed of this setpoint
    
    var oPOVRightTrigger = new Trigger(() -> operatorHID.getPOV() == 90);
    oPOVRightTrigger.onTrue(Commands.runOnce(() -> shooter.modifyShooterSpeedSetpoint(0.05)));

    var oPOVLeftTrigger = new Trigger(() -> operatorHID.getPOV() == 270);
    oPOVLeftTrigger.onTrue(Commands.runOnce(() -> shooter.modifyShooterSpeedSetpoint(-0.05)));

    // Source Intake

    var oRightTriggerTrigger = new Trigger(() -> operatorHID.getRightTriggerAxis() > 0.5);
    oRightTriggerTrigger.whileTrue(new RunSourceIntakeCmd(shooter));




    // CLIMBER COMMANDS
    
    var oLeftYAxisUp = new Trigger(() -> operatorHID.getLeftY() > 0.05);
    oLeftYAxisUp.whileTrue(new ClimberCmd(climber, () -> operatorHID.getLeftY()));

    var oLeftYAxisDown = new Trigger(() -> operatorHID.getLeftY() < -0.05);
    oLeftYAxisDown.whileTrue(new ClimberCmd(climber, () -> operatorHID.getLeftY()));

    // HOOK COMMANDS
    var oRightAxis = new Trigger(() -> Math.abs(operatorHID.getRightX()) > 0.05);
    oRightAxis.whileTrue(new MoveHookCmd(hook, operatorHID::getRightX));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Shoot into the speaker



    return autoCreator.createAuto();
  }

    
}