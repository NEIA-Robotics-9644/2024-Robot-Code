package frc.robot.subsystems.shooter.shooterAngle;

public interface ShooterAngleIO {
    

    public void setAngleDeg(double angle);

    public double getAngleDeg();

    /*
     * Updates angle, MUST BE CALLED EACH CYCLE
     
     */
    public void periodic();



    /*
     * Enable manual control of the mechanism
     * This disables all internal control loops
     * USE WITH CAUTION
     */
    public void setManualControl(boolean enabled);

    /*
     * Set the velocity of the mechanism
     * Negative is always down.
     * This is only used when manual control is enabled
     */
    public void setManualVelocity(double velocity);


    /*
     * Get whether manual control is enabled
     */
    public boolean manualControlEnabled();

    public boolean atBottom();

    public boolean atTop();

    public void resetAngleToBottom();


    public double getTopAngleDeg();

    public double getBottomAngleDeg();

    public double getVelocityPercent();


}
