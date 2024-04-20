package frc.robot.config;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;

/**
 * Configuration for a Swerve Module Custom Swerve Module Config for Team 6907, this current version
 * is from the 2024 robot, "Krait".
 */
public class SwerveModuleConfig {
  /** the CAN ID for Drive Motor */
  public int driveID = 0;

  /** The CAN ID for Azimuth Motor */
  public int azimuthID = 0;

  /** the DIO Port Number on RoboRIO */
  public int lightGateID = 0;

  /**
   * Falcon Absolute Encoder reading (through Tuner Self-snapshot) when pointing forward
   *
   * @unit rotation. best in range [0, 1)
   */
  public double azimuthEncoderOffsetRotation = 0;

  /**
   * The (roughly estimated) module position when the lightgate detect carbon plate. We will use the
   * section [centerDegree - 180.0 / RATIO, centerDegree + 180.0 / RATIO] to calibrate module and
   * add offset to azimuthEncoderOffsetRotation
   *
   * @unit degree. best be positive
   */
  public double azimuthBlockCenterDegree = 0;

  public InvertedValue invertDrive = InvertedValue.CounterClockwise_Positive;
  public InvertedValue invertAzimuth = InvertedValue.CounterClockwise_Positive;

  /** The PIDF constants for Drive Motor */
  public double DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF;

  /** The PIDF constants for Azimuth Motor */
  public double AZIMUTH_KP, AZIMUTH_KI, AZIMUTH_KD, AZIMUTH_KF;

  /**
   * Which corner the module is at. Contains `ModuleCorner.modulePosision`, which is the position
   * relative to the robot, from wheel center to robot center.
   */
  public ModuleCorner corner = null;

  public enum ModuleCorner {
    Front_Left(
        "FL", new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2)),
    Front_Right(
        "FR", new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kWheelBase / 2)),
    Back_Left(
        "BL", new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2)),
    Back_Right(
        "BR", new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kWheelBase / 2));

    public String name;
    public Translation2d modulePosition;

    ModuleCorner(String name, Translation2d modulePosition) {
      this.name = name;
      this.modulePosition = modulePosition;
    }
  }
}
