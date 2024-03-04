package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;

public class IntakeExtenderMechanismIOSim implements IntakeExtenderMechanismIO {
    

    // Simulate a rotating mechanism with top and bottom limits

    private double retractLimit = 90;
    private double extendLimit = 0;

    private double angle = 0;

    private boolean extended = false;

    private final PIDController motorModel = new PIDController(0.1, 0.0, 0.0);


    @Override
    public void periodic() {
        if (extended) {
            motorModel.setSetpoint(extendLimit);
        } else {
            motorModel.setSetpoint(retractLimit);
        }
        angle += motorModel.calculate(angle);
        angle = Math.max(extendLimit, Math.min(retractLimit, angle));
    }


    @Override
    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    @Override
    public double getAngleDeg() {
        return angle;
    }

    @Override
    public boolean getExtended() {
        return extended;
    }

    




}
