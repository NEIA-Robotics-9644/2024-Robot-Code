// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Modes mode = Modes.REAL;
  
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final DriveSubsystem driveSubsystem;
  
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (mode) {
      case SIM:
        driveSubsystem = new DriveSubsystem(
          new ModuleIOSim(0),
          new ModuleIOSim(1),
          new ModuleIOSim(2),
          new ModuleIOSim(3),
          new GyroIOSim()
        );
        break;
      case REAL:
        driveSubsystem = new DriveSubsystem(
          new ModuleIOSparkMax(0),
          new ModuleIOSparkMax(1),
          new ModuleIOSparkMax(2),
          new ModuleIOSparkMax(3),
          new GyroIOPigeon2()
        );
        break;

      default:
        throw new IllegalArgumentException("Invalid mode");
    }
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
    // Drive Command
    driveSubsystem.setDefaultCommand(
      new SwerveDriveCmd(
        driveSubsystem,
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX(),
        () -> !driverController.a().getAsBoolean()
      )
    );
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
