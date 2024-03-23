package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final IntakeExtenderMechanism extender;
    private final IntakeWheelMotor intakeWheel;

    private final boolean extenderInverted = false;
    private final boolean intakeWheelInverted = false;


    public IntakeSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public IntakeSubsystem(IntakeExtenderMechanismIO extender, IntakeWheelMotorIO intakeWheel) {
        if (extender == null || intakeWheel == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }
        this.extender = new IntakeExtenderMechanism(extender);
        this.intakeWheel = new IntakeWheelMotor(intakeWheel);

        this.intakeWheel.setInverted(intakeWheelInverted);
        // this.extender.setInverted(extenderInverted);
    }


    @Override
    public void periodic() {
        extender.periodic();
        intakeWheel.periodic();
    }


    public void deploy(boolean deployed) {
        
    }

    public void setExtended(boolean extended) {
        extender.setExtended(extended);
    }


    public void runFeeder(boolean reversed) {
        intakeWheel.runMotorAtPercentVelocity(reversed ? -1.0 : 1.0);
    }

    public double getIntakeWheelsPercentVelocity() {
        return intakeWheel.getMotorVelocityPercent();
    }

    public void setIntakeWheelsBrake(boolean brake) {
        intakeWheel.setBrake(brake);
    }

    public boolean getExtended() {
        return extender.getExtended();
    }

    public double getExtendAngleDeg() {
        return extender.getAngleDeg();
    }

    
}
