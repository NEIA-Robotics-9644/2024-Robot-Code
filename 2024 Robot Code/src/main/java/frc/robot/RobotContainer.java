// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.Utils;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Modes;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.JoystickDriveCmd;
import frc.robot.commands.MoveShooterToBottomAndResetCmd;
import frc.robot.commands.MoveShooterToSetpointCmd;
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

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

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

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  
  private final SmartDashboardDisplay display;
  private final String auto;

  private final PowerDistribution m_power;

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
          new ShooterWheelIOSparkMax(21),
          new ShooterWheelIOTalonFX(22),
          new FeederWheelIOSparkMax(23),
          new ShooterAngleIOSparkMax(24, 25),
          new NoteSensorIORoboRio(),
          new double[] { 0, 30, 60, 100},
          new double[] { 1, 0.4, 1, 0.4},
          new double[] { 1, 0.4, 1, 0.4}
      );

      climber = new ClimberSubsystem(
          new ClimberMotorIOSparkMax(26),
          new ClimberMotorIOSparkMax(27)
      ); 

    } else {
      shooter = new ShooterSubsystem(
          new ShooterWheelIOSim(),
          new ShooterWheelIOSim(),
          new FeederWheelIOSim(),
          new ShooterAngleIOSim(),
          new NoteSensorIOSim(),
          new double[] { 0, 30, 60, 100},
          new double[] { 1, 0.4, 1, 0.4},
          new double[] { 1, 0.4, 1, 0.4}
      );

      climber = new ClimberSubsystem(
          new ClimberMotorIOSim(),
          new ClimberMotorIOSim()
      );
    }
    
    m_power = new PowerDistribution();
    
    
    drivetrain.getPigeon2().reset();

    display = new SmartDashboardDisplay(drivetrain, shooter, climber, autoChooser);



    // Configure the trigger bindings
    configureBindings();
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
    driverHID.getLeftTriggerAxis(Logger.recordInput("Joystick", () -> driverHID.getPOV()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }


    

    // SHOOTER COMMANDS

    var oLeftTrigger = new Trigger(() -> operatorHID.getLeftTriggerAxis() > 0.5);
    oLeftTrigger.whileTrue(new SpinShooterWheelsCmd(shooter));
    oLeftTrigger.whileTrue(Logger.recordInput("Left Trigger", () -> operatorHID.getLeftTriggerAxis() > 0.5));
    
    var oLeftBumper = new Trigger(() -> operatorHID.getLeftBumper());
    oLeftBumper.whileTrue(new ShootWhenReadyCmd(shooter, 0.9, 0.8));
    oLeftBumper.whileTrue(Logger.recordInput("Left Bumper", () -> operatorHID.getLeftBumper()));

    // Bottom
    var oATrigger = new Trigger(() -> operatorHID.getAButton());
    oATrigger.onTrue(Commands.runOnce(() -> shooter.goToSetpoint(0)));
    oATrigger.onTrue(Logger.recordInput("A Button", () -> operatorHID.getAButton()));

    // Protected Shot
    var oBTrigger = new Trigger(() -> operatorHID.getBButton());
    oBTrigger.onTrue(Commands.runOnce(() -> shooter.goToSetpoint(1)));
    oBTrigger.onTrue(Logger.recordInput("B Button", () -> operatorHID.getBButton()));

    // Source Intake
    var oXTrigger = new Trigger(() -> operatorHID.getXButton());
    oXTrigger.onTrue(Commands.runOnce(() -> shooter.goToSetpoint(2)));
    oXTrigger.onTrue(Logger.recordInput("X Button", () -> operatorHID.getXButton()));

    // Speaker
    var oYTrigger = new Trigger(() -> operatorHID.getYButton());
    oYTrigger.onTrue(Commands.runOnce(() -> shooter.goToSetpoint(3)));
    oYTrigger.onTrue(Logger.recordInput("Y Button", () -> operatorHID.getYButton()));
    

    // Reset Shooter Angle
    var oStartTrigger = new Trigger(() -> operatorHID.getStartButton());
    oStartTrigger.whileTrue(new MoveShooterToBottomAndResetCmd(shooter, 1));
    oStartTrigger.whileTrue(Logger.recordInput("Start Button", () -> operatorHID.getStartButton()));

    // Adjust the shooter angle of this setpoint
    operatorController.povUp().onTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(1)));
    operatorController.povDown().onTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(-1)));

    var oPOVUpTrigger = new Trigger(() -> operatorHID.getPOV() == 0);
    oPOVUpTrigger.whileTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(1)));
    oPOVUpTrigger.whileTrue(Logger.recordInput("DPad Up", () -> operatorHID.getPOV() == 0));

    var oPOVDownTrigger = new Trigger(() -> operatorHID.getPOV() == 180);
    oPOVDownTrigger.whileTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(-1)));
    oPOVDownTrigger.whileTrue(Logger.recordInput("DPad Down", () -> operatorHID.getPOV() == 180));


    // Adjust the shooter wheel speed of this setpoint
    
    var oPOVRightTrigger = new Trigger(() -> operatorHID.getPOV() == 90);
    oPOVRightTrigger.whileTrue(Commands.runOnce(() -> shooter.modifyShooterSpeedSetpoint(0.05)));
    oPOVRightTrigger.whileTrue(Logger.recordInput("DPad Right", () -> operatorHID.getPOV() == 90));

    var oPOVLeftTrigger = new Trigger(() -> operatorHID.getPOV() == 270);
    oPOVLeftTrigger.whileTrue(Commands.runOnce(() -> shooter.modifyShooterSpeedSetpoint(-0.05)));
    oPOVLeftTrigger.whileTrue(Logger.recordInput("DPad Left", () -> operatorHID.getPOV() == 270));

    // Source Intake

    var oRightTriggerTrigger = new Trigger(() -> operatorHID.getRightTriggerAxis() > 0.5);
    oRightTriggerTrigger.whileTrue(new RunSourceIntakeCmd(shooter));
    oRightTriggerTrigger.whileTrue(Logger.recordInput("Right Trigger", () -> operatorHID.getRightTriggerAxis() > 0.5));



    // CLIMBER COMMANDS
    
    var oLeftYAxisUp = new Trigger(() -> operatorHID.getLeftY() > 0.05);
    oLeftYAxisUp.whileTrue(new ClimberCmd(climber, () -> operatorHID.getLeftY()));
    oLeftYAxisUp.whileTrue(Logger.recordInput("Left Axis Up", () -> operatorHID.getLeftY() > 0.05));

    var oLeftYAxisDown = new Trigger(() -> operatorHID.getLeftY() < -0.05);
    oLeftYAxisDown.whileTrue(new ClimberCmd(climber, () -> operatorHID.getLeftY()));
    oLeftYAxisDown.whileTrue(Logger.recordInput("Left Axis Down", () -> operatorHID.getLeftY() < -0.05));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    auto = autoChooser.getSelected();
    switch (auto) {
      case "MoveForward":
        return new SequentialCommandGroup(
          new JoystickDriveCmd(drivetrain, 3.0, 0.0, 0.0, false, false, true, false).withTimeout(3)
        );
        break;
      case "MoveBackward":
        return new SequentialCommandGroup(
          new JoystickDriveCmd(drivetrain, -3.0, 0.0, 0.0, false, false, true, false).withTimeout(3)
        );
        break;
      case "MoveLeft":
        return new SequentialCommandGroup(
          new JoystickDriveCmd(drivetrain, 0, 3, 0, false, false, true, false).withTimeout(3)
        );
        break;
      case "MoveRight":
        return new SequentialCommandGroup(
          new JoystickDriveCmd(drivetrain, 0, -3, 0, false, false, true, false).withTimeout(3)
        );
        break;
      case "Shoot":
      default:
        return new SequentialCommandGroup(
          new MoveShooterToBottomAndResetCmd(shooter, 1).withTimeout(1.75),
          new MoveShooterToSetpointCmd(shooter, 2).withTimeout(4.0),
          new SpinShooterWheelsCmd(shooter).withTimeout(1.5),
          new ShootWhenReadyCmd(shooter, 0.1, 0.99).withTimeout(1)
        );
        break;
      case "TestAuto":
        return new PathPlannerAuto("TestAuto");
        break;
      case "EdgeLoopAuto":
        return new PathPlannerAuto("EdgeLoopAuto");
        break;
      case "ShootPlusMoveToNoteIndex2":
        return new PathPlannerAuto("Shoot+MoveToNote2Auto");
        break;
      case "TurnAndShootAuto":
        return new PathPlannerAuto("TurnAndShootAuto");
        break;
    }
     
  }

  public PowerDistribution getPDH()
  {
    return m_power;
  }
}