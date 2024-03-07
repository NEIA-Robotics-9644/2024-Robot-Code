package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterAngleManagerCmd extends Command {
    
    private final ShooterSubsystem shooter;


    // A, B, X, Y
    private final double[] angleSetpoints;

    private final double trimAmountDeg;


    private int angleIndex = 0;

    private final Supplier<Boolean> aSupplier;
    private final Supplier<Boolean> bSupplier;
    private final Supplier<Boolean> xSupplier;
    private final Supplier<Boolean> ySupplier;
    private final Supplier<Boolean> trimUpSupplier;
    private final Supplier<Boolean> trimDownSupplier;

     
    /*
     * @param shooter The shooter subsystem to control
     * @param aSupplier A supplier for the A button
     * @param bSupplier A supplier for the B button
     * @param xSupplier A supplier for the X button
     * @param ySupplier A supplier for the Y button
     * @param trimUpSupplier A supplier for the trim up button
     * @param trimDownSupplier A supplier for the trim down button
     * @param angleSetpoints An array of angle setpoints for the shooter.  There should be 4 setpoints in the array, in the order A, B, X, Y.  By default, the setpoints should be {0, 40, 55, 70}
     * @param trimAmountDeg The amount of degrees to trim the angle setpoints by
     */
    public ShooterAngleManagerCmd(ShooterSubsystem shooter, Supplier<Boolean> aSupplier, Supplier<Boolean> bSupplier,
            Supplier<Boolean> xSupplier, Supplier<Boolean> ySupplier,
            Supplier<Boolean> trimUpSupplier, Supplier<Boolean> trimDownSupplier, double[] angleSetpoints, double trimAmountDeg) {
        this.shooter = shooter;
        this.aSupplier = aSupplier;
        this.bSupplier = bSupplier;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.trimUpSupplier = trimUpSupplier;
        this.trimDownSupplier = trimDownSupplier;
        this.angleSetpoints = angleSetpoints;
        this.trimAmountDeg = trimAmountDeg;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        
        if (aSupplier.get()) {
            angleIndex = 0;
        } else if (bSupplier.get()) {
            angleIndex = 1;
        } else if (xSupplier.get()) {
            angleIndex = 2;
        } else if (ySupplier.get()) {
            angleIndex = 3;
        }

        shooter.setShooterAngleDeg(angleSetpoints[angleIndex]);

        if (trimUpSupplier.get()) {
            angleSetpoints[angleIndex] += trimAmountDeg;
        } else if (trimDownSupplier.get()) {
            angleSetpoints[angleIndex] -= trimAmountDeg;
        }
    }
}
