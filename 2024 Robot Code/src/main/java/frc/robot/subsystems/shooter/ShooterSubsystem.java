package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterMotor rightShooterWheel;
    private final ShooterMotor leftShooterWheel;

    private final boolean rightShooterWheelReversed = false;
    private final boolean leftShooterWheelReversed = false;

    private final ShooterAngleMechanism angleMechanism;
    
    
    private final ShooterMotor feeder;

    private final boolean feederReversed = true;

    public ShooterSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterSubsystem(ShooterMotorIO leftShooter, ShooterMotorIO rightShooter, ShooterMotorIO feeder, ShooterAngleIO angleMechanism) {

        if (leftShooter == null || rightShooter == null || feeder == null || angleMechanism == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize things
        this.leftShooterWheel = new ShooterMotor(leftShooter);
        this.rightShooterWheel = new ShooterMotor(rightShooter);
        this.feeder = new ShooterMotor(feeder);
        this.angleMechanism = new ShooterAngleMechanism(angleMechanism);

        // Turn on brake mode for the feeder
        this.feeder.setBrake(true);
    }
    @Override
    public void periodic() {
        rightShooterWheel.periodic();
        leftShooterWheel.periodic();
        feeder.periodic();

        angleMechanism.periodic();
        
    }
    

    


    /*
     * Spin the shooter wheels
     */
    public void spinShooterWheels(boolean reversed) {
        if (reversed) {
            leftShooterWheel.setMotorVelocity(leftShooterWheelReversed ? -1.0 : 1.0);
            rightShooterWheel.setMotorVelocity(rightShooterWheelReversed ? 1.0 : -1.0);
        } else {
            leftShooterWheel.setMotorVelocity(leftShooterWheelReversed ? 1.0 : -1.0);
            rightShooterWheel.setMotorVelocity(rightShooterWheelReversed ? -1.0 : 1.0);
        }
    }

    /*
     * Get the percent at which the shooter wheels are spinning
     */
    public double getShooterWheelsSpeedPercent() {
        return (leftShooterWheel.getMotorPercentSpeed() + rightShooterWheel.getMotorPercentSpeed()) / 2.0;
    }

    /*
     * Spin the feeder wheel
     */
    public void spinFeederWheel(boolean reversed) {
        if (reversed) {
            feeder.setMotorVelocity(feederReversed ? -1.0 : 1.0);
        } else {
            feeder.setMotorVelocity( feederReversed ? 1.0 : -1.0);
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
}