package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.DriveSubsystem;

public class BrakeAllCmd extends Command{
    private final DriveSubsystem driveSubsystem;


    public BrakeAllCmd(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        driveSubsystem.brakeModules();
    }
}
