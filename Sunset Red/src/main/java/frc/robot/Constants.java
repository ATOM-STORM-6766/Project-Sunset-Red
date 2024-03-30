// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.config.SwerveModuleConfig;
import frc.robot.config.SwerveModuleConfig.ModuleCorner;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kPeriodicDt = TimedRobot.kDefaultPeriod; // seconds

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveModuleConstants {
    public static final SwerveModuleConfig FL, FR, BL, BR;

    static {
      FL = new SwerveModuleConfig();
      FL.driveID = 4;
      FL.azimuthID = 5;
      FL.azimuthBlockCenterDegree = 105.0;
      FL.azimuthEncoderOffsetRotation = 0.508789;
      FL.lightGateID = 0;
      FL.corner = ModuleCorner.Front_Left;
      FL.invertDrive = InvertedValue.CounterClockwise_Positive;
      FL.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      FL.DRIVE_KP = 0.3;
      FL.DRIVE_KI = 0.0;
      FL.DRIVE_KD = 0.0;
      FL.DRIVE_KF = 0.11;
      FL.AZIMUTH_KP = 12.0;
      FL.AZIMUTH_KI = 0.0;
      FL.AZIMUTH_KD = 0.1;
      FL.AZIMUTH_KF = 0.11;

      FR = new SwerveModuleConfig();
      FR.driveID = 6;
      FR.azimuthID = 7;
      FR.azimuthBlockCenterDegree = 105.0;
      FR.azimuthEncoderOffsetRotation = 0.456543;
      FR.lightGateID = 3;
      FR.corner = ModuleCorner.Front_Right;
      FR.invertDrive = InvertedValue.CounterClockwise_Positive;
      FR.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      FR.DRIVE_KP = 0.3;
      FR.DRIVE_KI = 0.0;
      FR.DRIVE_KD = 0.0;
      FR.DRIVE_KF = 0.11;
      FR.AZIMUTH_KP = 12.0;
      FR.AZIMUTH_KI = 0.0;
      FR.AZIMUTH_KD = 0.1;
      FR.AZIMUTH_KF = 0.11;

      BL = new SwerveModuleConfig();
      BL.driveID = 2;
      BL.azimuthID = 3;
      BL.azimuthBlockCenterDegree = 105.0;
      BL.azimuthEncoderOffsetRotation = 0.689453;
      BL.lightGateID = 1;
      BL.corner = ModuleCorner.Back_Left;
      BL.invertDrive = InvertedValue.Clockwise_Positive;
      BL.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      BL.DRIVE_KP = 0.3;
      BL.DRIVE_KI = 0.0;
      BL.DRIVE_KD = 0.0;
      BL.DRIVE_KF = 0.11;
      BL.AZIMUTH_KP = 12.0;
      BL.AZIMUTH_KI = 0.0;
      BL.AZIMUTH_KD = 0.1;
      BL.AZIMUTH_KF = 0.11;

      BR = new SwerveModuleConfig();
      BR.driveID = 0;
      BR.azimuthID = 1;
      BR.azimuthBlockCenterDegree = 105.0;
      BR.azimuthEncoderOffsetRotation = 0.684082;
      BR.lightGateID = 2;
      BR.corner = ModuleCorner.Back_Right;
      BR.invertDrive = InvertedValue.Clockwise_Positive;
      BR.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      BR.DRIVE_KP = 0.3;
      BR.DRIVE_KI = 0.0;
      BR.DRIVE_KD = 0.0;
      BR.DRIVE_KF = 0.11;
      BR.AZIMUTH_KP = 12.0;
      BR.AZIMUTH_KI = 0.0;
      BR.AZIMUTH_KD = 0.1;
      BR.AZIMUTH_KF = 0.11;
    }
  }

  public static final class DriveConstants {
    // driver constants
    public static final double kDeadband = 0.1;

    // chassis dimensions
    public static final double kTrackWidth = 0.56;
    public static final double kWheelBase = 0.56;
    public static final double kChassisWheelDiameterMeters = 0.13; // meters
    public static final double kChassisWheelCircumferenceMeters =
        kChassisWheelDiameterMeters * Math.PI; // meters

    // Pigeon constants
    public static final int kPigeonPort = 0;

    // physical robot constants
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond =
        kPhysicalMaxSpeedMetersPerSecond * 0.9;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond * 0.9;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class DebugConstants {
    public static final double kLongCANTimeoutSec = 0.1;
    // Update Freq (Hz): minimum 4Hz, maximum 1000Hz
    public static final int kOdomUpdateFreq = 100; // signal for odometry
    public static final int kDefaultUpdateFreq = 50;
  }
}
