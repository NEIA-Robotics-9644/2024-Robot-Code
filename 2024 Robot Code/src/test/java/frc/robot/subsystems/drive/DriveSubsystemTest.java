package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;

import net.bytebuddy.asm.Advice.OffsetMapping.Factory.Illegal;

import static org.mockito.Mockito.mock;;


public class DriveSubsystemTest {
    

    @Test
    public void itShouldThrowAnExceptionIfNotPassedValidHardware() {
        assertThrows(
            IllegalArgumentException.class,
            () -> {
                DriveSubsystem driveSubsystem = new DriveSubsystem();
            });
    }

    @Test
    public void itShouldThrowAnExceptionIfPassedNullHardware() {
        assertThrows(
            IllegalArgumentException.class,
            () -> {
                DriveSubsystem driveSubsystem =
                    new DriveSubsystem(null, null, null, null, null);
            });
    }

    @Test
    public void itShouldUpdateGyroInputs() {
        GyroIO gyroIO = mock(GyroIO.class);
        ModuleIO flModuleIO = mock(ModuleIO.class);
        ModuleIO frModuleIO = mock(ModuleIO.class);
        ModuleIO brModuleIO = mock(ModuleIO.class);
        ModuleIO blModuleIO = mock(ModuleIO.class);

        DriveSubsystem driveSubsystem =
            new DriveSubsystem(gyroIO, flModuleIO, frModuleIO, brModuleIO, blModuleIO);
        
        
        driveSubsystem.periodic();


}
