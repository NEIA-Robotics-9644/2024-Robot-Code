package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ModuleIOSim implements ModuleIO {
    
    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(0);

    private final int index;

    
    public ModuleIOSim(int index) {
        this.index = index;
    }

    @Override
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    }

    @Override
    public double getDriveVelocity() {
        return driveSim.getAngularVelocityRPM();
    }

    
  @Override
  public void setDriveVoltage(double volts) {
    var driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    var turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        System.out.println("Module " + index + " Drive Brake Mode: " + enable);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        System.out.println("Module " + index + " Turn Brake Mode: " + enable);
    }

    @Override
    public void periodic() {
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Log data to SmartDashboard
        SmartDashboard.putNumber("Module " + index + " Drive Velocity (mps)", getDriveVelocity());
        SmartDashboard.putNumber("Module " + index + " Turn Absolute Position (deg)", getAbsoluteRotation().getDegrees());
    }

    @Override
    public double getTurnP() {
        return 0.0;
    }

    @Override
    public double getTurnI() {
        return 0.0;
    }

    @Override
    public double getTurnD() {
        return 0.0;
    }
}
