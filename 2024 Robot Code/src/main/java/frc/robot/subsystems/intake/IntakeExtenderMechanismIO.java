package frc.robot.subsystems.intake;


public interface IntakeExtenderMechanismIO {
    

    public void periodic();

    public void setExtended(boolean extended);

    public double getAngleDeg();

    public boolean getExtended();
}
