// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum Modes {
    SIM,
    REAL
  }

  public static final class CANBusIDs {
    public static final int kPigeon2CanID = 14;
  }

  public static final class PhysicalRobotCharacteristics {
    public static final double kWheelBaseMeters = Units.inchesToMeters(26);
    public static final double kTrackWidthMeters = Units.inchesToMeters(26);

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(-kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0),    // Front Left
            new Translation2d(kTrackWidthMeters / 2.0, kTrackWidthMeters / 2.0),    // Front Right
            new Translation2d(-kTrackWidthMeters / 2.0, -kWheelBaseMeters / 2.0),   // Back Left
            new Translation2d(kTrackWidthMeters / 2.0, -kWheelBaseMeters / 2.0)     // Back Right
        };

    public static final double kWheelRadiusMeters = Math.hypot(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0);

    public static final double kMaxLinearSpeedMetersPerSec = Units.feetToMeters(15);

    public static final double kMaxAngularSpeedRadPerSec = kMaxLinearSpeedMetersPerSec / kWheelRadiusMeters;
  }

    
}
