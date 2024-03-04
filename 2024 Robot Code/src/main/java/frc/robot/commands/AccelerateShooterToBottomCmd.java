package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AccelerateShooterToBottomCmd extends Command {

    private final ShooterSubsystem shooter;
    private final double maxSpeedDegPerSec;

    private double lastPositionDeg = 0.0;

    public AccelerateShooterToBottomCmd(ShooterSubsystem shooter, double maxSpeedDegPerSec) {
        this.maxSpeedDegPerSec = maxSpeedDegPerSec;

        this.shooter = shooter;

        

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        lastPositionDeg = shooter.getShooterAngleDeg();
        System.out.println("AccelerateShooterToBottomCmd initialized");

        // Also send the shooter to the bottom
        shooter.setShooterAngleDeg(shooter.getShooterBottomAngleDeg());
    }

    @Override
    public void execute() {

        shooter.setManualAngleControl(true);


        // Get the shooter's current velocity

        // TODO: THIS IS BAD CODE. DO NOT DO THIS.  IF THIS ISN'T CALLED EACH CYCLE, IT WILL NOT WORK
        double currentVelocityDegPerSec = (shooter.getShooterAngleDeg() - lastPositionDeg) / 0.02;

        // Velocity is negative
        double newVelocity = Math.max(-maxSpeedDegPerSec, currentVelocityDegPerSec - 10.0);

        shooter.setManualAngleVelocityDegPerSec(newVelocity);
        
        
        lastPositionDeg = shooter.getShooterAngleDeg(); 


        System.out.println("Accelerating towards bottom.  Shooter Velocity: " + newVelocity + " deg/sec");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setManualAngleControl(false);

        if (shooter.atBottom()) {
            shooter.resetAngleToBottom();
        }

        System.out.println("AccelerateShooterToBottomCmd " + (interrupted ? "interrupted" : "ended") + " at " + shooter.getShooterAngleDeg() + " degrees.  Shooter at bottom: " + shooter.atBottom());
    }

    @Override
    public boolean isFinished() {
        if (shooter.atBottom()) {
            System.out.println("Shooter at bottom");
        }
        return shooter.atBottom();
    }
    
}
