package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterWheelIO rightShooterWheel;
    private final ShooterWheelIO leftShooterWheel;

    private final boolean rightShooterWheelReversed = false;
    private final boolean leftShooterWheelReversed = false;

    private final ShooterAngleIO angleMechanism;

    private final double[] angleSetpoints;

    private final double[] wheelSpeedSetpoints;

    private final double[] feederSpeedSetpoints;

    private int setpointIndex = 0;
    
    
    private final FeederWheelIO feeder;

    private final boolean feederReversed = true;

    private final NoteSensorIO noteSensor;


    public ShooterSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    /*
     * Create a new shooter subsystem
     * @param leftShooter The left shooter wheel
     * @param rightShooter The right shooter wheel
     * @param feeder The feeder wheel
     * @param angleMechanism The angle mechanism
     * @param noteSensor The note sensor
     * @param angleSetpoints in degrees
     * @param wheelSpeedSetpoints normalized
     * @param feederSpeedSetpoints normalized
     */
    public ShooterSubsystem(ShooterWheelIO leftShooter, ShooterWheelIO rightShooter, FeederWheelIO feeder, ShooterAngleIO angleMechanism, NoteSensorIO noteSensor, double[] angleSetpoints
    , double[] wheelSpeedSetpoints, double[] feederSpeedSetpoints) {

        if (leftShooter == null || rightShooter == null || feeder == null || angleMechanism == null || noteSensor == null || angleSetpoints == null || wheelSpeedSetpoints == null || feederSpeedSetpoints == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        if (angleSetpoints.length != wheelSpeedSetpoints.length || angleSetpoints.length != feederSpeedSetpoints.length) {
            throw new IllegalArgumentException("The length of the angle setpoints, wheel speed setpoints, and feeder speed setpoints must be the same");
        }

        // Initialize things
        this.leftShooterWheel = leftShooter;
        this.rightShooterWheel = rightShooter;
        this.feeder = feeder;
        this.angleMechanism = angleMechanism;
        this.noteSensor = noteSensor;


        this.wheelSpeedSetpoints = wheelSpeedSetpoints;

        this.angleSetpoints = angleSetpoints;
        this.feederSpeedSetpoints = feederSpeedSetpoints;
    

        // Turn on brake mode for the feeder
        this.feeder.setBrakeMode(true);

        
    }

    @Override
    public void periodic() {

        angleMechanism.setAngleDeg(angleSetpoints[setpointIndex]);
        
        angleMechanism.periodic();
        leftShooterWheel.periodic();
        rightShooterWheel.periodic();
        feeder.periodic();
        
        // Only do this if the robot is enabled, running teleoperated
        if (DriverStation.isEnabled()) {
            noteSensor.setDisplayLight(noteSensor.noteDetected());
        } else {
            noteSensor.setDisplayLight(false);
        }
        
    }


    /*
     * Spin the shooter wheels
     */
    public void spinShooterWheels(boolean reversed) {
        double speed = wheelSpeedSetpoints[setpointIndex];
        if (reversed) {
            leftShooterWheel.spinWheel(leftShooterWheelReversed ? -speed : speed);
            rightShooterWheel.spinWheel(rightShooterWheelReversed ? speed : -speed);
        } else {
            leftShooterWheel.spinWheel(leftShooterWheelReversed ? speed : -speed);
            rightShooterWheel.spinWheel(rightShooterWheelReversed ? -speed : speed);
        }
    }

    /*
     * Get the percent at which the shooter wheels are spinning
     */
    public double getShooterWheelsSpeedPercent() {
        return (Math.abs(leftShooterWheel.getVelocityPercent()) + Math.abs(rightShooterWheel.getVelocityPercent())) / 2.0;
    }

    public double getRightShooterWheelVelocityPercent() {
        return rightShooterWheel.getVelocityPercent();
    }

    public double getLeftShooterWheelVelocityPercent() {
        return leftShooterWheel.getVelocityPercent();
    }

    public double getFeederVelocityPercent() {
        return feeder.getVelocityPercent();
    }   


    /*
     * Spin the feeder wheel
     */
    public void spinFeederWheel(boolean reversed) {
        if (reversed) {
            feeder.spinWheel(feederReversed ? -feederSpeedSetpoints[setpointIndex] : feederSpeedSetpoints[setpointIndex]);
        } else {
            feeder.spinWheel( feederReversed ? feederSpeedSetpoints[setpointIndex] : -feederSpeedSetpoints[setpointIndex]);
        }
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


    /*
     * Move to a specific setpoint
     * This is used to change the shooter angle, wheel speed, and feeder speed
     */
    public void goToSetpoint(int setpointIndex) {
        if (setpointIndex < 0 || setpointIndex >= angleSetpoints.length) {
            throw new IllegalArgumentException("Invalid setpoint.  Setpoint must be between 0 and " + (angleSetpoints.length - 1) + " inclusive.");
        }
        this.setpointIndex = setpointIndex;
    }


    /*
     * Modify the current setpoint
     */
    public void modifyAngleSetpoint(double delta) {
        angleSetpoints[setpointIndex] += delta;
    }
    

    public void modifyFeederSetpoint(double delta) {
        feederSpeedSetpoints[setpointIndex] += delta;
    }

    public void modifyShooterSpeedSetpoint(double delta) {
        wheelSpeedSetpoints[setpointIndex] += delta;
    }

    public int getSetpointIndex() {
        return setpointIndex;
    }

    /*
     * Degrees
     */
    public double getAngleSetpoint() {
        return angleSetpoints[setpointIndex];
    }

    /*
     * Normalized wheel speed setpoint
     
     */
    public double getWheelSpeedSetpoint() {
        return wheelSpeedSetpoints[setpointIndex];
    }

    /*
     * Normalized feeder speed setpoint
     */
    public double getFeederSpeedSetpoint() {
        return feederSpeedSetpoints[setpointIndex];
    }

}
