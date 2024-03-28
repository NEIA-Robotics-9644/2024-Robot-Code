package frc.robot.subsystems.hook;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hook.hookMotor.HookMotorIO;;

public class HookSubsystem extends SubsystemBase{

    private final HookMotorIO hook;

    public HookSubsystem() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public HookSubsystem(HookMotorIO hook) {

        if (hook == null) {
            throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
        }

        this.hook = hook;

        
    }
    @Override
    public void periodic() {
        
        hook.periodic();


    }

    
    public void moveHook(double normalizedVelocity) {
        hook.spinMotor(normalizedVelocity);
    }

    public double getHookRotations() {
        return (hook.getMotorRotations()) / 2;
    }

    public double getHookSpeed() {
        return (Math.abs(hook.getMotorVelocityRPM())) / 2;
    }
}
