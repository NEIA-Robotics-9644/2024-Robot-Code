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
import frc.robot.Constants.Modes;
import frc.robot.commands.TestMotorSpinCmd;
import frc.robot.subsystems.testMotorComponent.TestMotorSubsystem;
import frc.robot.subsystems.testMotorComponent.testMotor.TestMotorIOSim;
import frc.robot.subsystems.testMotorComponent.testMotor.TestMotorIOSparkMax;

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
  private final CommandXboxController mainController = new CommandXboxController(0);
  
  private final TestMotorSubsystem testMotorSubsystem;
 
  
  private final SmartDashboardDisplay display;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Utils.isSimulation()) {
      mode = Modes.SIM;
    } else {
      mode = Modes.REAL;
    }

    // Constants.DriveConstants.putValues();

    if (mode == Modes.REAL) {
      testMotorSubsystem = new TestMotorSubsystem(
          new TestMotorIOSparkMax(21)
      );

    } else {

      testMotorSubsystem = new TestMotorSubsystem(
          new TestMotorIOSim()
      );

    }
    


    display = new SmartDashboardDisplay(testMotorSubsystem);

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

    var mainHID = mainController.getHID();
    
    var oLeftXAxisUp = new Trigger(() -> mainHID.getLeftX() > 0.05);
    oLeftXAxisUp.whileTrue(new TestMotorSpinCmd(testMotorSubsystem, () -> mainHID.getLeftY()));

    var oLeftXAxisDown = new Trigger(() -> mainHID.getLeftX() < -0.05);
    oLeftXAxisDown.whileTrue(new TestMotorSpinCmd(testMotorSubsystem, () -> mainHID.getLeftY()));
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    
}

