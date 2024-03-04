// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Modes;
import frc.robot.commands.AccelerateShooterToBottomCmd;
import frc.robot.commands.JoystickDriveCmd;
import frc.robot.commands.MoveShooterToSetpointCmd;
import frc.robot.commands.ShootWhenReadyCmd;
import frc.robot.commands.SpinShooterWheelsCmd;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterMotorIOSparkMax.ShooterMotorType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Modes mode = Modes.REAL;
  
  // private double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  // private double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = driverController;
  private final SwerveDriveSubsystem drivetrain = DriveConstants.DriveTrain; // My drivetrain
  
  private final ShooterSubsystem shooter = new ShooterSubsystem(
    new frc.robot.subsystems.shooter.ShooterMotorIOSparkMax(ShooterMotorType.LEFT_FLYWHEEL), 
    new frc.robot.subsystems.shooter.ShooterMotorIOSparkMax(ShooterMotorType.RIGHT_FLYWHEEL), 
    new frc.robot.subsystems.shooter.ShooterMotorIOSparkMax(ShooterMotorType.FEEDER),
    new frc.robot.subsystems.shooter.ShooterAngleIOSparkMax(24, 25)
  );
  
  private final IntakeSubsystem intake = new IntakeSubsystem(
    new frc.robot.subsystems.intake.MotorIOSparkMax(0), 
    new frc.robot.subsystems.intake.MotorIOSparkMax(1));
  
  
  private final SmartDashboardDisplay display = new SmartDashboardDisplay(drivetrain, shooter, intake);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    

    drivetrain.getPigeon2().reset();


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
    drivetrain.setDefaultCommand(new JoystickDriveCmd(drivetrain, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // drivetrain.registerTelemetry(logger::telemeterize);

    
    

    // SHOOTER COMMANDS

    

    operatorController.leftTrigger().whileTrue(new SpinShooterWheelsCmd(shooter));

    operatorController.leftBumper().whileTrue(new ShootWhenReadyCmd(shooter, 0.9, 0.8));

    operatorController.a().onTrue(new AccelerateShooterToBottomCmd(shooter, 20));  
    operatorController.b().onTrue(new MoveShooterToSetpointCmd(shooter, 45.0));
    operatorController.x().onTrue(new MoveShooterToSetpointCmd(shooter, 90.0));
    operatorController.y().onTrue(new MoveShooterToSetpointCmd(shooter, 135.0));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
