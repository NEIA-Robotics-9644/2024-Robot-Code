// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.Utils;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Modes;
import frc.robot.commands.AutoMoveRobotCentricCmd;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.JoystickDriveCmd;
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
import frc.robot.subsystems.shooter.shooterWheel.ShooterWheelIOSparkMax;
import frc.robot.subsystems.shooter.shooterWheel.ShooterWheelIOTalonFX;
import frc.robot.subsystems.shooter.shooterWheel.ShooterWheelIOSim;
import frc.robot.subsystems.shooter.shooterAngle.ShooterAngleIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.noteSensor.NoteSensorIORoboRio;
import frc.robot.subsystems.shooter.noteSensor.NoteSensorIOSim;




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
          new ShooterWheelIOSparkMax(21),
          new ShooterWheelIOTalonFX(22),
          new FeederWheelIOSparkMax(23),
          new ShooterAngleIOSparkMax(24, 25),
          new NoteSensorIORoboRio(),
          new double[] { 0, 25, 52, 52},
          new double[] { 1, 0.4, 1, 1},
          new double[] { 1, 0.4, 1, 1}
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
          new double[] { 0, 25, 48, 52},
          new double[] { 1, 0.4, 1, 1},
          new double[] { 1, 0.4, 1, 0.4}
      );

      climber = new ClimberSubsystem(
          new ClimberMotorIOSim(),
          new ClimberMotorIOSim()
      );

    }
    


    display = new SmartDashboardDisplay(drivetrain, shooter, climber);

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
    oLeftTrigger.whileTrue(new SpinShooterWheelsCmd(shooter, operatorController));

    var oLeftBumper = new Trigger(() -> operatorHID.getLeftBumper());
    oLeftBumper.whileTrue(new ShootWhenReadyCmd(shooter, 0.001, 0.0001));

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
    oYTrigger.onTrue(Commands.runOnce(() -> shooter.goToSetpoint(3)));
    
    // Reset Shooter Angle
    var oStartTrigger = new Trigger(() -> operatorHID.getStartButton());
    oStartTrigger.whileTrue(new MoveShooterToBottomAndResetCmd(shooter, 0.2));

    // Adjust the shooter angle of this setpoint
    var oPOVUpTrigger = new Trigger(() -> operatorHID.getPOV() == 0);
    oPOVUpTrigger.onTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(1)));
    
    var oPOVDownTrigger = new Trigger(() -> operatorHID.getPOV() == 180);
    oPOVDownTrigger.onTrue(Commands.runOnce(() -> shooter.modifyAngleSetpoint(-1)));
    


    // Adjust the shooter wheel speed of this setpoint
    
    var oPOVRightTrigger = new Trigger(() -> operatorHID.getPOV() == 90);
    oPOVRightTrigger.onTrue(Commands.runOnce(() -> shooter.modifyShooterSpeedSetpoint(0.01)));

    var oPOVLeftTrigger = new Trigger(() -> operatorHID.getPOV() == 270);
    oPOVLeftTrigger.onTrue(Commands.runOnce(() -> shooter.modifyShooterSpeedSetpoint(-0.01)));

    // Adjust the feeder wheel speed of this setpoint
    var oRightUpTrigger = new Trigger(() -> operatorHID.getRightY() > 0.7);
    oRightUpTrigger.onTrue(Commands.runOnce(() -> shooter.modifyFeederSetpoint(-0.01)));

    var oRightDownTrigger = new Trigger(() -> operatorHID.getRightY() < -0.7);
    oRightDownTrigger.onTrue(Commands.runOnce(() -> shooter.modifyFeederSetpoint(0.01)));


    // Source Intake

    var oRightTriggerTrigger = new Trigger(() -> operatorHID.getRightTriggerAxis() > 0.5);
    oRightTriggerTrigger.whileTrue(new RunSourceIntakeCmd(shooter));




    // CLIMBER COMMANDS
    
    var oLeftYAxisUp = new Trigger(() -> operatorHID.getLeftY() > 0.05);
    oLeftYAxisUp.whileTrue(new ClimberCmd(climber, () -> operatorHID.getLeftY()));

    var oLeftYAxisDown = new Trigger(() -> operatorHID.getLeftY() < -0.05);
    oLeftYAxisDown.whileTrue(new ClimberCmd(climber, () -> operatorHID.getLeftY()));
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    

    return autoCreator.createAuto();
  }

  public Command getTestCommand() {
    return new SequentialCommandGroup(
      Commands.runOnce(() -> System.out.println("STARTING TEST \nPRESS A TO MOVE TO NEXT TEST")),
      Commands.runOnce(() -> System.out.println("Driving Forward")),
      new AutoMoveRobotCentricCmd(drivetrain, 1, 0, 0).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Turning")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      new AutoMoveRobotCentricCmd(drivetrain, 0, 0, 30).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Shooter Angle Reset")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      new MoveShooterToBottomAndResetCmd(shooter, 0.05).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Shooter Angle Setpoint 0")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      Commands.run(() -> shooter.goToSetpoint(0)).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Shooter Angle Setpoint 1")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      Commands.run(() -> shooter.goToSetpoint(1)).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Shooter Angle Setpoint 2")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      Commands.run(() -> shooter.goToSetpoint(2)).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Shooter Angle Setpoint 3")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      Commands.run(() -> shooter.goToSetpoint(3)).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Spin Feeder Wheel")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      new ShootWhenReadyCmd(shooter, -0.01, -0.1).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Spin Shooter Wheels")),
      new WaitUntilCommand(() -> !driverController.a().getAsBoolean() && !operatorController.a().getAsBoolean()),
      new SpinShooterWheelsCmd(shooter, operatorController).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean()),
      Commands.runOnce(() -> System.out.println("Manually Move Climbers Up and Down")),
      new ClimberCmd(climber, () -> -operatorController.getLeftY()).until(() -> driverController.a().getAsBoolean() || operatorController.a().getAsBoolean())
           

    );
  }

    
}

