package frc.robot.subsystems.shooter;


public class ShooterAngleMechanism {
    
    private final ShooterAngleIO shooterAngleIO;

    private double angleSetpoint = 0;
    

    public ShooterAngleMechanism() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public ShooterAngleMechanism(ShooterAngleIO shooterAngleIO) {
        if (shooterAngleIO == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        this.shooterAngleIO = shooterAngleIO;
        
    }

    /*
     * Updates angle, MUST BE CALLED EACH CYCLE
     */
    public void periodic() {

        shooterAngleIO.periodic();
    }

    public void setAngleDeg(double angle) {
        shooterAngleIO.setAngleDeg(angle);
        angleSetpoint = angle;
    }

    public double getAngleDeg() {
        return shooterAngleIO.getAngleDeg();
    }

    public boolean atSetpoint() {
        return shooterAngleIO.atSetpoint();
    }

    public double getSetpoint() {
        return angleSetpoint;
    }

    public void setManualControl(boolean enabled) {
        shooterAngleIO.setManualControl(enabled);
    }

    public boolean manualControlEnabled() {
        return shooterAngleIO.manualControlEnabled();
    }

    public void setManualVelocityDegPerSec(double velocity) {
        shooterAngleIO.setManualVelocityDegPerSec(velocity);
    }

    public boolean atBottom() {
        return shooterAngleIO.atBottom();
    }

    public boolean atTop() {
        return shooterAngleIO.atTop();
    }

    public void resetAngleToBottom() {
        shooterAngleIO.resetAngleToBottom();
    }

    public double getTopAngleDeg() {
        return shooterAngleIO.getTopAngleDeg();
    }

    public double getBottomAngleDeg() {
        return shooterAngleIO.getBottomAngleDeg();
    }
 
}
