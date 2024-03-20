// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveModuleConstants{
    public static final double kWheelDiameterMeters = 0.13;
    public static final double kDriveMotorGearRatio = 106.0 / 11.0;
    public static final double kSteerMotorGearRatio = 6.0;
    public static final double kDriveEncoderRot2Meter = kWheelDiameterMeters * Math.PI / kDriveMotorGearRatio;
    public static final double kSteerEncoderRot2Rad = 2 * Math.PI / kSteerMotorGearRatio;
    public static final double kDriveEncoderRPM2MeterPerSecond = (kWheelDiameterMeters * Math.PI / kDriveMotorGearRatio) * (1 / 60.0); // Adjusted to convert RPM to meters per second correctly
    public static final double kSteerEncoderRPM2RadPerSecond = 2 * Math.PI / 60.0 / kSteerMotorGearRatio; // Correct as it converts RPM to rad/s
    public static final double kPsteer = 12.0;
  }

  public static final class DriveConstants {
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
  }
  

}
