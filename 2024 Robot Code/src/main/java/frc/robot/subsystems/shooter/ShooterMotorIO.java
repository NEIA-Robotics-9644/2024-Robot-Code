package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterMotorIO {

    /*
     * Use setMotorVelocity instead
     */
    public void setMotorVoltage(double voltage);


    /*
     * Set the velocity of the motor
     * Normalized value between -1 and 1
     */
    public void setMotorVelocity(double velocity);

    /*
     * Get the maximum voltage the motor can be set to
     * Remember that setMotorVelocity() exists
     */
    public double getMaxMotorVoltage();

    public Rotation2d getAbsoluteRotation();

    public void setBrake(boolean brake);

    /*
     * Get the velocity of the motor
     * In RPM
     */
    public double getMotorVelocityRPM();

    /*
     * Get the percent speed of the motor
     * Normalized value between 0 and 1
     * Based on the motor IO's internal max speed
     */
    public double getMotorPercentSpeed();

    public void periodic();
}
