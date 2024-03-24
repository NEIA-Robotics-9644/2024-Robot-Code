package frc.robot.subsystems.shooter.noteSensor;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class NoteSensorIORoboRio implements NoteSensorIO{
    // Initializes a DigitalInput on DIO 5
    private DigitalInput input = new DigitalInput(5);

    private PowerDistribution pdh;

    public NoteSensorIORoboRio() {
        this.pdh = new PowerDistribution(1, ModuleType.kRev);
    }


    @Override

    public boolean noteDetected() {
        // Returns the value of the DigitalInput
        

        return input.get();
    }

    @Override
    public void setDisplayLight(boolean on) {
        pdh.setSwitchableChannel(on);
    }
}
