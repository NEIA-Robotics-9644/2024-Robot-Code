package frc.robot.subsystems.servoMotorComponent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.servoMotorComponent.testServo.TestServoIO;

public class TestServoSubsystem extends SubsystemBase{

    private final TestServoIO testServo;

    public TestServoSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public TestServoSubsystem(TestServoIO servo) {

        if (servo == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        this.testServo = servo;
    }
    
    public void rotateTestServo(double angle) {
        testServo.setServoAngle(angle);
    }

    public double getTestServoAngle() {
        return (testServo.getServoAngle());
    }
}
