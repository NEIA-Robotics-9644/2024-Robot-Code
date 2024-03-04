package frc.robot.subsystems.intake;

public class IntakeWheelMotor {

    private final IntakeWheelMotorIO motor;

    boolean inverted;

    public IntakeWheelMotor(IntakeWheelMotorIO motor) {
        if (motor == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }
        this.motor = motor;
    }

    public void runMotorAtPercentVelocity(double percentVelocity) {
        motor.runMotorAtPercentVelocity(inverted ? -percentVelocity : percentVelocity);
    }

    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    public double getMotorVelocityPercent() {
        return inverted ? -motor.getMotorVelocityPercent() : motor.getMotorVelocityPercent();
    }

    public void periodic() {
        motor.periodic();
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
}
