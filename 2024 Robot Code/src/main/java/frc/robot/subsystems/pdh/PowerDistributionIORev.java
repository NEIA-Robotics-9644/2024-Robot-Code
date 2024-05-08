package frc.robot.subsystems.pdh;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.pdh.PowerDistributionIO;

public class PowerDistributionIORev implements PowerDistributionIO{

    private final PowerDistribution PDH;
    
    public PowerDistributionIORev() {
        throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
    }

    public PowerDistributionIORev(int CanID) {
        this.PDH = new PowerDistribution(CanID, ModuleType.kRev);
    }

    @Override
    public double channelCurrent(int Channel)
    {
        return PDH.getCurrent(Channel);
    }
    
    @Override
    public double totalPower()
    {
        return PDH.getTotalPower();
    }
}
