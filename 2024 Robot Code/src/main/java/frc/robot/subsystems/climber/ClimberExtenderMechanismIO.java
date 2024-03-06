package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberExtenderMechanismIO {
    public void periodic();
    public void setExtended(boolean extended);
    public boolean getExtended();
    public double getAngle();
}
