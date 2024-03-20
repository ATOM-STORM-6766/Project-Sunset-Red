// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveModuleConstants {
    public static final double kWheelDiameterMeters = 0.13;
    public static final double kDriveMotorGearRatio = 106.0 / 11.0;
    public static final double kSteerMotorGearRatio = 6.0;
    public static final double kDriveEncoderRot2Meter = kWheelDiameterMeters * Math.PI / kDriveMotorGearRatio;
    public static final double kSteerEncoderRot2Rad = 2 * Math.PI / kSteerMotorGearRatio;
    public static final double kDriveEncoderRPM2MeterPerSecond = (kWheelDiameterMeters * Math.PI / kDriveMotorGearRatio)
        * (1 / 60.0); // Adjusted to convert RPM to meters per second correctly
    public static final double kSteerEncoderRPM2RadPerSecond = 2 * Math.PI / 60.0 / kSteerMotorGearRatio; // Correct as
                                                                                                          // it converts
                                                                                                          // RPM to
                                                                                                          // rad/s
    public static final double kPsteer = 12.0;
  }

  public static final class DriveConstants {
    // chassis dimensions
    public static final double kTrackWidth = 0.56;
    public static final double kWheelBase = 0.56;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // front left module
    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kFrontLeftTurningMotorPort = 5;
    public static final boolean kFrontLeftSteerMotorReversed = false;
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final double kFrontLeftSteerEncoderOffset = -0.254; // radians
    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
    // front right module
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kFrontRightTurningMotorPort = 7;
    public static final boolean kFrontRightSteerMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final double kFrontRightSteerEncoderOffset = -0.254; // radians
    public static final int kFrontRightAbsoluteEncoderPort = 1;
    public static final boolean kFrontRightAbsoluteEncoderReversed = false;

    // back left module
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kBackLeftTurningMotorPort = 3;
    public static final boolean kBackLeftSteerMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final double kBackLeftSteerEncoderOffset = -0.254; // radians
    public static final int kBackLeftAbsoluteEncoderPort = 2;
    public static final boolean kBackLeftAbsoluteEncoderReversed = false;


    // back right module
    public static final int kBackRightDriveMotorPort = 0;
    public static final int kBackRightTurningMotorPort = 1;
    public static final boolean kBackRightSteerMotorReversed = false;
    public static final boolean kBackRightDriveMotorReversed = false;
    public static final double kBackRightSteerEncoderOffset = -0.254; // radians
    public static final int kBackRightAbsoluteEncoderPort = 3;
    public static final boolean kBackRightAbsoluteEncoderReversed = false;

    // physical robot constants
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

  }

}
