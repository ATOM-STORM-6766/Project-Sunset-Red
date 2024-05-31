package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.config.SwerveModuleConfig;
import frc.robot.utils.Util;

public class SwerveDriveModule {
  private static final double AZIMUTH_GEAR_RATIO = 6.0;
  private static final double DRIVE_GEAR_RATIO = 106.0 / 11.0;

  private final TalonFX mDriveMotor;
  private final TalonFX mAzimuthMotor;
  private final DigitalInput mLightGate;

  private SwerveModuleConfig mConfig;

  // whether zeroing has complete
  private boolean isZeroed = false;

  /* Drive motor control requests */
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* Azimuth motor control requests */
  private final MotionMagicVoltage azimuthPosition = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut azimuthVolt = new VoltageOut(0);

  // logEntries

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
        mAzimuthMotor
            .getConfigurator()
            .apply(mAzimuthConfigs, OdometryConstants.kLongCANTimeoutSec));
    mAzimuthMotor.setControl(new NeutralOut());
    Util.checkReturn(
        "swerve azimuth position",
        mAzimuthMotor
            .getPosition()
            .setUpdateFrequency(
                OdometryConstants.kOdomUpdateFreq, OdometryConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve azimuth velocity",
        mAzimuthMotor
            .getVelocity()
            .setUpdateFrequency(
                OdometryConstants.kOdomUpdateFreq, OdometryConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve azimuth canbus",
        mAzimuthMotor.optimizeBusUtilization(OdometryConstants.kLongCANTimeoutSec));

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
        mDriveMotor.getConfigurator().apply(mDriveConfig, OdometryConstants.kLongCANTimeoutSec));
    mDriveMotor.setControl(new NeutralOut());
    Util.checkReturn(
        "swerve drive position",
        mDriveMotor
            .getPosition()
            .setUpdateFrequency(
                OdometryConstants.kOdomUpdateFreq, OdometryConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve drive velocity",
        mDriveMotor
            .getVelocity()
            .setUpdateFrequency(
                OdometryConstants.kOdomUpdateFreq, OdometryConstants.kLongCANTimeoutSec));
    Util.checkReturn(
        "swerve drive canbus",
        mDriveMotor.optimizeBusUtilization(OdometryConstants.kLongCANTimeoutSec));

    mDriveMotor.setPosition(0);
  }

  private double azimuthErr() {
    double errrot = azimuthPosition.Position - mAzimuthMotor.getPosition().getValueAsDouble();
    // round to [-180, 180]
    double errdeg = 360.0 * (errrot - Math.floor(errrot));
    if (errdeg > 180.0) {
      errdeg -= 360.0;
    }

    return errdeg;
  }

  public void initSendable(SendableBuilder builder, String namePrefix) {
    final String name = "/" + mConfig.corner.name;
    builder.addBooleanProperty(name + "/isZeroed", () -> isZeroed, null);
    builder.addDoubleProperty(name + "/VelocityMs", () -> getDriveVelocityMs(), null);
    builder.addDoubleProperty(
        name + "/VelocityErrorMs",
        () ->
            DriveConstants.kChassisWheelCircumferenceMeters
                * (driveVelocity.Velocity - mDriveMotor.getVelocity().getValueAsDouble()),
        null);
    builder.addDoubleProperty(
        name + "/AzimuthAngleDeg", () -> getAzimuthAngleRotations() * 360.0, null);
    builder.addDoubleProperty(name + "/AzimuthErrorDeg", () -> azimuthErr(), null);
  }

  public void initLogEntry(String namePrefix) {
    // logEntries construct here
  }

  public void updateLogEntry() {
    // logEntries append here
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
    double setpointRotations = goal.getRotations() + mConfig.azimuthEncoderOffsetRotation;
    mAzimuthMotor.setControl(azimuthPosition.withPosition(setpointRotations));
  }

  /**
   * Sets the drive velocity of the swerve drive module.
   *
   * @param velocityMetersPerSecond the desired velocity in meters per second
   */
  public void setDriveVelocity(double velocityMetersPerSecond) {
    double targetRPS =
        velocityMetersPerSecond
            / DriveConstants.kChassisWheelCircumferenceMeters; // no need to adjust for gear ratio
    // because it's already done in the
    // TalonFX configuration
    mDriveMotor.setControl(driveVelocity.withVelocity(targetRPS));
  }

  public double getDriveVelocityMs() {
    // Convert RPS to MPS, no need to adjust for gear ratio because it's already
    // done in the TalonFX configuration
    return mDriveMotor.getVelocity().getValueAsDouble()
        * DriveConstants.kChassisWheelCircumferenceMeters;
  }

  public double getDriveDistanceMeters() {
    // Convert motor rotations to meters, no need to adjust for gear ratio because
    // it's already done in the TalonFX configuration
    // This assumes DRIVE_GEAR_RATIO defines motor rotations per wheel rotation.
    return mDriveMotor.getPosition().getValueAsDouble()
        * DriveConstants.kChassisWheelCircumferenceMeters;
  }

  public double getAzimuthAngleRotations() {
    return mAzimuthMotor.getPosition().getValueAsDouble() - mConfig.azimuthEncoderOffsetRotation;
  }

  /**
   * Returns the state of the swerve drive module. The state includes the velocity in meters per
   * second and the azimuth rotation in radians.
   *
   * @return the state of the swerve drive module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocityMs(), Rotation2d.fromRotations(getAzimuthAngleRotations()));
  }

  /**
   * Represents the position of a swerve drive module. The position includes the distance traveled
   * in meters and the azimuth rotation in radians.
   */
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        getDriveDistanceMeters(), Rotation2d.fromRotations(getAzimuthAngleRotations()));
  }

  /**
   * Sets the voltage for the azimuth motor of the swerve drive module.
   *
   * @param volt the voltage to be set for the azimuth motor
   */
  public void setAzimuthVoltage(double volt) {
    mAzimuthMotor.setControl(azimuthVolt.withOutput(volt));
  }

  public void startZeroing() {
    setAzimuthVoltage(1.5);
  }

  public boolean checkLightGate() {
    return !mLightGate.get(); // depend on the connection + -
  }

  /**
   * Stops the motor and calibrates the module based on current pos sets the motor to neutral and
   * adjusts the azimuth encoder offset rotation to align with the desired block center degree
   */
  public void stopAndCalibrate() {
    // stop the motor and calibrate based on current position
    mAzimuthMotor.setControl(new NeutralOut());
    double azimuthPosition = mAzimuthMotor.getPosition().getValueAsDouble();
    double remainer =
        mConfig.azimuthEncoderOffsetRotation
            + Units.degreesToRotations(mConfig.azimuthBlockCenterDegree)
            - (0.5 / AZIMUTH_GEAR_RATIO);
    mConfig.azimuthEncoderOffsetRotation +=
        Math.floor((azimuthPosition - remainer) * AZIMUTH_GEAR_RATIO) / AZIMUTH_GEAR_RATIO;
    isZeroed = true;
  }

  public boolean getIsZeroed() {
    return isZeroed;
  }

  public Translation2d getTranslationToRobotCenter() {
    return mConfig.corner.modulePosition;
  }

  public StatusSignal<Double> getAzimuthPositionSignal() {
    return mAzimuthMotor.getPosition();
  }

  public StatusSignal<Double> getAzimuthVelocitySignal() {
    return mAzimuthMotor.getVelocity();
  }

  public StatusSignal<Double> getDrivePositionSignal() {
    return mDriveMotor.getPosition();
  }

  public StatusSignal<Double> getDriveVelocitySignal() {
    return mDriveMotor.getVelocity();
  }

  public double getAzimuthOffsetRotations() {
    return mConfig.azimuthEncoderOffsetRotation;
  }

  /** Module position relative to chassis center, robot oriented */
  public Translation2d getModulePositionToCenter() {
    return mConfig.corner.modulePosition;
  }
}
