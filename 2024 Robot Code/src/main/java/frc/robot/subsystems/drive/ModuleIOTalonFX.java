package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ModuleIOTalonFX implements ModuleIO {

    // Gear ratios for SDS MK4i L2
    private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double TURN_GEAR_RATIO = 150.0 / 7.0;
    
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder turnAbsoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;
    
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;

    private final int index;

    private final boolean isTurnMotorInverted;

    public ModuleIOTalonFX() {
        throw new IllegalArgumentException("You must pass in valid hardware");
    }

    public ModuleIOTalonFX(int index) {
        this.index = index;
        switch (index) {
            case 0:
                // FRONT LEFT
                driveTalon = new TalonFX(1);
                turnTalon = new TalonFX(2);
                turnAbsoluteEncoder = new CANcoder(3);
                absoluteEncoderOffset = new Rotation2d(-0.293213 * 2 * Math.PI);  // TODO calibrate on new robot
                isTurnMotorInverted = false;
                break;
            case 1:
                // FRONT RIGHT
                driveTalon = new TalonFX(4);
                turnTalon = new TalonFX(5);
                turnAbsoluteEncoder = new CANcoder(6);
                absoluteEncoderOffset = new Rotation2d(0.292236 * 2 * Math.PI);  // TODO calibrate on new robot
                isTurnMotorInverted = false;
                break;
            case 2:
                // BACK LEFT
                driveTalon = new TalonFX(7);
                turnTalon = new TalonFX(8);
                turnAbsoluteEncoder = new CANcoder(9);
                absoluteEncoderOffset = new Rotation2d(-0.260254 * 2 * Math.PI);  // TODO calibrate on new robot
                isTurnMotorInverted = false;
                break;
            case 3:
                // BACK RIGHT
                driveTalon = new TalonFX(10);
                turnTalon = new TalonFX(11);
                turnAbsoluteEncoder = new CANcoder(12);
                absoluteEncoderOffset = new Rotation2d(-0.227051 * 2 * Math.PI);  // TODO calibrate on new robot
                isTurnMotorInverted = false;
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnTalon.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        turnAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        turnAbsolutePosition = turnAbsoluteEncoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, drivePosition, turnPosition); // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);
        driveTalon.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // Log data to SmartDashboard
        SmartDashboard.putNumber("Module " + index + " Drive Velocity", getDriveVelocity());
        SmartDashboard.putNumber("Module " + index + " Turn Absolute Position", turnAbsoluteEncoder.getPosition().getValueAsDouble());
    }

    @Override
    public double getDriveVelocity() {
        return driveVelocity.getValueAsDouble() / DRIVE_GEAR_RATIO;
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d(
            (turnAbsoluteEncoder.getPosition().getValueAsDouble()
                    - Math.floor(turnAbsoluteEncoder.getPosition().getValueAsDouble()))
                * 2.0
                * Math.PI)
        .minus(absoluteEncoderOffset);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted =
            isTurnMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnTalon.getConfigurator().apply(config);
    }

    @Override
    public double getTurnP() {
        return 5.0;
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
