package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.testMotorComponent.TestMotorSubsystem;

public class SmartDashboardDisplay extends SubsystemBase {

    private final TestMotorSubsystem testMotor;



    public SmartDashboardDisplay(TestMotorSubsystem motor) {    
        this.testMotor = motor;
    }

    public void periodic() {
        SmartDashboard.putNumber("Test Motor Rotation (Degrees)", testMotor.getTestMotorRotations());
    }
    
}
