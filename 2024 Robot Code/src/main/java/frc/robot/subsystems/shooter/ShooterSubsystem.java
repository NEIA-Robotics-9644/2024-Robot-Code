package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterWheelIO rightShooterWheel;
    private final ShooterWheelIO leftShooterWheel;

    private final boolean rightShooterWheelReversed = false;
    private final boolean leftShooterWheelReversed = false;

    private final ShooterAngleMechanism angleMechanism;
    
    
    private final FeederWheelIO feeder;

    private final boolean feederReversed = true;

    private final NoteSensorIO noteSensor;


    public ShooterSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterSubsystem(ShooterWheelIO leftShooter, ShooterWheelIO rightShooter, FeederWheelIO feeder, ShooterAngleIO angleMechanism, NoteSensorIO noteSensor) {

        if (leftShooter == null || rightShooter == null || feeder == null || angleMechanism == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.leftShooterWheel = leftShooter;
        this.rightShooterWheel = rightShooter;
        this.feeder = feeder;
        this.angleMechanism = new ShooterAngleMechanism(angleMechanism);
        this.noteSensor = noteSensor;

        // Turn on brake mode for the feeder
        this.feeder.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        

        angleMechanism.periodic();
        leftShooterWheel.periodic();
        rightShooterWheel.periodic();
        feeder.periodic();
        

        
    }


    /*
     * Spin the shooter wheels
     */
    public void spinShooterWheels(boolean reversed) {
        if (reversed) {
            leftShooterWheel.spinWheel(leftShooterWheelReversed ? -1.0 : 1.0);
            rightShooterWheel.spinWheel(rightShooterWheelReversed ? 1.0 : -1.0);
        } else {
            leftShooterWheel.spinWheel(leftShooterWheelReversed ? 1.0 : -1.0);
            rightShooterWheel.spinWheel(rightShooterWheelReversed ? -1.0 : 1.0);
        }
    }

    /*
     * Get the percent at which the shooter wheels are spinning
     */
    public double getShooterWheelsSpeedPercent() {
        return (leftShooterWheel.getSpeedPercent() + rightShooterWheel.getSpeedPercent()) / 2.0;
    }

    /*
     * Set the speed of the shooter wheel
     */
    public double getFeederSpeedPercent() {
        return feeder.getSpeedPercent();
    }   


    /*
     * Spin the feeder wheel
     */
    public void spinFeederWheel(boolean reversed) {
        if (reversed) {
            feeder.spinWheel(feederReversed ? -1.0 : 1.0);
        } else {
            feeder.spinWheel( feederReversed ? 1.0 : -1.0);
        }
    }


    /*
     * Set the desired angle of the shooter in degrees
     */
    public void setShooterAngleDeg(double angle) {
        angleMechanism.setAngleDeg(angle);
    }


    /*
     * Get the current angle of the shooter in degrees
     */
    public double getShooterAngleDeg() {
        return angleMechanism.getAngleDeg();
    }


    /*
     * Get whether the shooter has satisfactorally converged on the desired angle
     */
    public boolean atSetpoint() {
        return angleMechanism.atSetpoint();
    }

    public double getShooterAngleSetpointDeg() {
        return angleMechanism.getSetpoint();
    }


    /*
     * Enable manual control of the angle mechanism
     * This disables all internal control loops
     * USE WITH CAUTION
     */
    public void setManualAngleControl(boolean enabled) {
        angleMechanism.setManualControl(enabled);
    }


    /*
     * Get whether manual control of the angle mechanism is enabled
     */
    public boolean manualAngleControlEnabled() {
        return angleMechanism.manualControlEnabled();
    }


    /*
     * Set the velocity of the angle mechanism
     * This is only used when manual control is enabled
     */
    public void setManualAngleVelocityDegPerSec(double velocity) {
        angleMechanism.setManualVelocityDegPerSec(velocity);
    }    


    /*
     * Get whether the shooter is at the bottom of its travel
     */
    public boolean atBottom() {
        return angleMechanism.atBottom();
    }

    /*
     * Get whether the shooter is at the top of its travel
     */
    public boolean atTop() {
        return angleMechanism.atTop();
    }

    /*
     * Reset the angle of the shooter to the bottom
     */
    public void resetAngleToBottom() {
        angleMechanism.resetAngleToBottom();
    }


    public double getShooterTopAngleDeg() {
        return angleMechanism.getTopAngleDeg();
    }

    public double getShooterBottomAngleDeg() {
        return angleMechanism.getBottomAngleDeg();
    }


    public boolean noteDetected() {
        return noteSensor.noteDetected();
    }
}