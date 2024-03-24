package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.SwerveModuleConfig;
import frc.robot.utils.Util;

public class SwerveDriveModule {
  private static final double AZIMUTH_GEAR_RATIO = 6.0;
  private static final double DRIVE_GEAR_RATIO = 106.0 / 11.0;

  private final TalonFX mDriveMotor;
  private final TalonFX mAzimuthMotor;
  private final DigitalInput mLightGate;

  private SwerveModuleConfig mConfig;

  /* Drive motor control requests */
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* Azimuth motor control requests */
  private final MotionMagicVoltage azimuthPosition = new MotionMagicVoltage(0).withSlot(0);

  public ControlRequest azimuthSelectedControl =
      azimuthPosition; // default to position control, can be changed to
  // velocity or duty cycle

  public SwerveDriveModule(SwerveModuleConfig config) {
    mDriveMotor = new TalonFX(config.driveID);
    mAzimuthMotor = new TalonFX(config.azimuthID);
    mConfig = config;
    mLightGate = new DigitalInput(config.lightGateID);

    var mAzimuthConfigs = new TalonFXConfiguration();
    mAzimuthConfigs.MotorOutput.Inverted = mConfig.invertAzimuth;
    mAzimuthConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    mAzimuthConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    mAzimuthConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    mAzimuthConfigs.Voltage.PeakForwardVoltage = 12.0;
    mAzimuthConfigs.Voltage.PeakReverseVoltage = -12.0;
    mAzimuthConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    mAzimuthConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    mAzimuthConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    mAzimuthConfigs.CurrentLimits.SupplyCurrentLimit = 30.0;
    mAzimuthConfigs.CurrentLimits.SupplyCurrentThreshold = 30.0;
    mAzimuthConfigs.CurrentLimits.SupplyTimeThreshold = 0.0;
    // target error within 0.1
    mAzimuthConfigs.Slot0.kV = mConfig.AZIMUTH_KF;
    mAzimuthConfigs.Slot0.kP = mConfig.AZIMUTH_KP;
    mAzimuthConfigs.Slot0.kI = mConfig.AZIMUTH_KI;
    mAzimuthConfigs.Slot0.kD = mConfig.AZIMUTH_KD;
    mAzimuthConfigs.MotionMagic.MotionMagicJerk = 0.0;
    mAzimuthConfigs.MotionMagic.MotionMagicCruiseVelocity = 100 / AZIMUTH_GEAR_RATIO;
    mAzimuthConfigs.MotionMagic.MotionMagicAcceleration = 100 / AZIMUTH_GEAR_RATIO * 10.0;
    mAzimuthConfigs.ClosedLoopGeneral.ContinuousWrap =
        true; // This is important for azimuth control
    mAzimuthConfigs.Feedback.SensorToMechanismRatio = AZIMUTH_GEAR_RATIO;

    Util.checkReturn(
        "swerve azimuth",
        mAzimuthMotor.getConfigurator().apply(mAzimuthConfigs, DebugConstants.kLongCANTimeoutSec));
    mAzimuthMotor.setControl(new NeutralOut());
    Util.checkReturn(
        "swerve azimuth position",
        mAzimuthMotor
            .getPosition()
            .setUpdateFrequency(DebugConstants.kOdomUpdateFreq, DebugConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve azimuth velocity",
        mAzimuthMotor
            .getVelocity()
            .setUpdateFrequency(DebugConstants.kOdomUpdateFreq, DebugConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve azimuth canbus",
        mAzimuthMotor.optimizeBusUtilization(DebugConstants.kLongCANTimeoutSec));

    /* !todo Add logging and debugging info here */

    var mDriveConfig = new TalonFXConfiguration();
    mDriveConfig.MotorOutput.Inverted = mConfig.invertDrive;
    mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    mDriveConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    mDriveConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    mDriveConfig.Voltage.PeakForwardVoltage = 12.0;
    mDriveConfig.Voltage.PeakReverseVoltage = -12.0;
    mDriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    mDriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    mDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    mDriveConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    mDriveConfig.CurrentLimits.SupplyCurrentThreshold = 30.0;
    mDriveConfig.CurrentLimits.SupplyTimeThreshold = 0.0;
    mDriveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    // target error within 5
    mDriveConfig.Slot0.kV = mConfig.DRIVE_KF;
    mDriveConfig.Slot0.kP = mConfig.DRIVE_KP;
    mDriveConfig.Slot0.kI = mConfig.DRIVE_KI;
    mDriveConfig.Slot0.kD = mConfig.DRIVE_KD;
    Util.checkReturn(
        "swerve drive",
        mDriveMotor.getConfigurator().apply(mDriveConfig, DebugConstants.kLongCANTimeoutSec));
    mDriveMotor.setControl(new NeutralOut());
    Util.checkReturn(
        "swerve drive position",
        mDriveMotor
            .getPosition()
            .setUpdateFrequency(DebugConstants.kOdomUpdateFreq, DebugConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve drive velocity",
        mDriveMotor
            .getVelocity()
            .setUpdateFrequency(DebugConstants.kOdomUpdateFreq, DebugConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve drive canbus",
        mDriveMotor.optimizeBusUtilization(DebugConstants.kLongCANTimeoutSec));

    mDriveMotor.setPosition(0);
  }

  /**
   * Sets the desired state of the swerve drive module. The desired state includes the desired angle
   * and speed of the module. The method optimizes the desired state based on the current angle of
   * the module. It then sets the azimuth angle and drive velocity accordingly.
   *
   * @param desiredState The desired state of the swerve drive module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);
    setAzimuthDegree(optimizedState.angle);
    setDriveVelocity(optimizedState.speedMetersPerSecond);
  }

  /**
   * Sets the azimuth degree of the swerve drive module.
   *
   * @param goal The desired rotation angle in degrees.
   */
  public void setAzimuthDegree(Rotation2d goal) {
    double adjustedGoalRotations = goal.getRotations() + mConfig.azimuthEncoderOffsetRotation;
    mAzimuthMotor.setControl(azimuthPosition.withPosition(adjustedGoalRotations));
  }

  /**
   * Sets the drive velocity of the swerve drive module.
   *
   * @param velocityMetersPerSecond the desired velocity in meters per second
   */
  public void setDriveVelocity(double velocityMetersPerSecond) {
    double wheelCircumferenceMeters = DriveConstants.kChassisWheelDiameterMeters * Math.PI;
    double targetRPS =
        velocityMetersPerSecond / wheelCircumferenceMeters; // no need to adjust for gear ratio
    // because it's already done in the
    // TalonFX configuration
    mDriveMotor.setControl(driveVelocity.withVelocity(targetRPS));
  }

  /**
   * Returns the state of the swerve drive module. The state includes the velocity in meters per
   * second and the azimuth rotation in radians.
   *
   * @return the state of the swerve drive module
   */
  public SwerveModuleState getState() {

    // Convert RPS to MPS, no need to adjust for gear ratio because it's already
    // done in the TalonFX configuration
    double velocityMPS =
        mDriveMotor.getVelocity().getValueAsDouble()
            * DriveConstants.kChassisWheelDiameterMeters
            * Math.PI;

    return new SwerveModuleState(
        velocityMPS,
        Rotation2d.fromRotations(
            mAzimuthMotor.getPosition().getValueAsDouble() - mConfig.azimuthEncoderOffsetRotation));
  }

  /**
   * Represents the position of a swerve drive module. The position includes the distance traveled
   * in meters and the azimuth rotation in radians.
   */
  public SwerveModulePosition getPosition() {

    // Convert motor rotations to meters, no need to adjust for gear ratio because
    // it's already done in the TalonFX configuration
    // This assumes DRIVE_GEAR_RATIO defines motor rotations per wheel rotation.
    double distanceMeters =
        mDriveMotor.getPosition().getValueAsDouble()
            * DriveConstants.kChassisWheelDiameterMeters
            * Math.PI;

    return new SwerveModulePosition(
        distanceMeters,
        Rotation2d.fromRotations(
            mAzimuthMotor.getPosition().getValueAsDouble() - mConfig.azimuthEncoderOffsetRotation));
  }
}
