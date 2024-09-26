package frc.robot.subsystems.servoMotorComponent.testServo;

import edu.wpi.first.wpilibj.Servo;


public class TestServoIOReal implements TestServoIO {
    
    private final Servo servo;

    private double maxServoAngle = 270;

    public TestServoIOReal(int id) {
        this.servo = new Servo(id);
    }

    @Override
    public void setServoAngle(double angle) {
        servo.set(angle/maxServoAngle);
    }
    @Override
    public double getServoAngle() {
        return servo.get() * maxServoAngle;
    }
    @Override
    public void periodic() {

    }
}