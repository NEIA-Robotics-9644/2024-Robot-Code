package frc.robot.subsystems.intake;

public interface IntakeWheelMotorIO {
    

    /*
     * Set the motor velocity
     * Normalized input in the range -1 to 1
     */
    public void runMotorAtPercentVelocity(double percentVelocity);

    public void setBrake(boolean brake);

    public double getMotorVelocityPercent();

    /*
     * Periodic update
     * MUST BE CALLED EVERY CYCLE
     */
    public void periodic();

}
