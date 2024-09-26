package frc.robot.subsystems.testMotorComponent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.testMotorComponent.testMotor.TestMotorIO;;

public class TestMotorSubsystem extends SubsystemBase{

    private final TestMotorIO testMotor;

    public TestMotorSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public TestMotorSubsystem(TestMotorIO testMotor) {

        if (testMotor == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        this.testMotor = testMotor;

        
    }
    @Override
    public void periodic() {
        
        testMotor.periodic();
    }

    
    public void spinTestMotor(double normalizedVelocity) {
        testMotor.spinMotor(normalizedVelocity);
    }

    public double getTestMotorRotations() {
        return (testMotor.getMotorRotations());
    }

    public double getTestMotorSpeed() {
        return (Math.abs(testMotor.getMotorVelocityRPM()));
    }
}
