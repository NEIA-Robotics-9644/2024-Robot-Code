package frc.robot.subsystems.shooter;


import edu.wpi.first.wpilibj.DigitalInput;

public class NoteSensorIORoboRio implements NoteSensorIO{
    // Initializes a DigitalInput on DIO 5
    DigitalInput input = new DigitalInput(5);


    @Override

    public boolean noteDetected() {
        // Returns the value of the DigitalInput
        if (input.get()) {
            System.out.println("Note Detected");
        }

        return input.get();
    }
}
