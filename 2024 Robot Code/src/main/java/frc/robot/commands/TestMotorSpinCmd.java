package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.testMotorComponent.TestMotorSubsystem;

import java.util.function.Supplier;

/*
 * This is a command that will be used to drive the robot with a joystick
 * This command originated from the CTRE Phoenix Swerve Drive Generated Code
 */
public class TestMotorSpinCmd extends Command {

    private final TestMotorSubsystem testMotorSubsystem;

    private final Supplier<Double> velocity;

    
    
    public TestMotorSpinCmd(TestMotorSubsystem motor, Supplier<Double> vel) {
        
        testMotorSubsystem = motor;
        velocity = vel;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(motor);
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        testMotorSubsystem.spinTestMotor(velocity.get());
    }

    

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}