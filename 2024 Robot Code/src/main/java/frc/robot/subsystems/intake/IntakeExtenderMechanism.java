package frc.robot.subsystems.intake;


public class IntakeExtenderMechanism {

    private final IntakeExtenderMechanismIO io;

    
    public IntakeExtenderMechanism() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public IntakeExtenderMechanism(IntakeExtenderMechanismIO io) {
        // Check for null hardware
        if (io == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        // Initialize hardware
        this.io = io;
    }

    public void periodic() {
        io.periodic();
    }

    public void setExtended(boolean extended) {
        io.setExtended(extended);
    }

    public double getAngleDeg() {
        return io.getAngleDeg();
    }

    public boolean getExtended() {
        return io.getExtended();
    }

    
    
}
