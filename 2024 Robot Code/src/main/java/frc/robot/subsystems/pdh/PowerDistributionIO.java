package frc.robot.subsystems.pdh;

public interface PowerDistributionIO {
    public double channelCurrent(int Channel);
    public double totalPower();
}
