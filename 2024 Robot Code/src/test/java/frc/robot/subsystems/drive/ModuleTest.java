package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleTest {
    
    @Test
    void itShouldThrowAnExceptionIfPassedNull() {
        assertThrows(IllegalArgumentException.class, () -> {
            new Module();
        });
    }

    @Test
    void itShouldThrowAnExceptionIfNotPassedValidHardware() {
        assertThrows(IllegalArgumentException.class, () -> {
            new Module(null);
        });
    }

    @Test
    void itShouldCallPeriodicOnItsHardware() {
        ModuleIO io = mock(ModuleIO.class);

        Module module = new Module(io);

        module.periodic();

        verify(io).periodic();
    }

    @Test
    void itShouldSetTheDriveVoltageOfItsHardware() {
        ModuleIO io = mock(ModuleIO.class);

        when(io.getAbsoluteRotation()).thenReturn(new Rotation2d());

        Module module = new Module(io);

        module.drive(new SwerveModuleState(0, new Rotation2d()));

        verify(io).setDriveVoltage(Mockito.anyDouble());
    }

    @Test
    void itShouldSetTheTurnVoltageOfItsHardware() {
        ModuleIO io = mock(ModuleIO.class);

        when(io.getAbsoluteRotation()).thenReturn(new Rotation2d());

        Module module = new Module(io);

        module.drive(new SwerveModuleState(0, new Rotation2d()));

        verify(io).setTurnVoltage(Mockito.anyDouble());
    }
}
