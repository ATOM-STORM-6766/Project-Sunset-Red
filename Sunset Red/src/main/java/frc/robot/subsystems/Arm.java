package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Util;

public class Arm extends SubsystemBase {
  private static final double ARM_GEAR_RATIO = 80.0;
  // private static final double ARM_CLIMB_TARGET_DEG = 30.0;
  // private static final double ARM_CLIMB_MAX_DEG = 35.0;
  private static final double ERR_TOL = 1.0 / 360.0;

  private static final double LOW_SHOOT_TOLERANCE_ROTATION = 23.5 / 360.0;

  private final TalonFX mArmTalon;
  // private boolean climbTargetReached = false;
  private final SoftwareLimitSwitchConfigs mSoftLimitConf = new SoftwareLimitSwitchConfigs();

  private final MotionMagicVoltage ArmMotionMagic =
      new MotionMagicVoltage(ArmConstants.ARM_REST_POSITION);
  private final VoltageOut ArmVoltage = new VoltageOut(0);

  public Arm() {
    mArmTalon = new TalonFX(ArmConstants.ARM_ID);
    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    mSoftLimitConf.ForwardSoftLimitThreshold = ArmConstants.ARM_MAX_POSITION;
    mSoftLimitConf.ForwardSoftLimitEnable = true;
    mSoftLimitConf.ReverseSoftLimitThreshold = ArmConstants.ARM_REST_POSITION;
    mSoftLimitConf.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch = mSoftLimitConf;
    armConfig.Voltage.PeakForwardVoltage = 12.0;
    armConfig.Voltage.PeakReverseVoltage = -12.0;
    armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    armConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    armConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    armConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.kG = 0.7;
    armConfig.Slot0.kV = 9.028;
    armConfig.Slot0.kP = 100.0;
    armConfig.Slot0.kI = 0.0;
    armConfig.Slot0.kD = 0.0;
    armConfig.MotionMagic.MotionMagicJerk = 0.0;
    // seems mechanism rotation not rotor position
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 1.25;
    armConfig.MotionMagic.MotionMagicAcceleration = 2.0;
    armConfig.Feedback.SensorToMechanismRatio = ARM_GEAR_RATIO;
    Util.checkReturn(
        "arm", mArmTalon.getConfigurator().apply(armConfig, Constants.kLongCANTimeoutSec));
    mArmTalon.setPosition(ArmConstants.ARM_REST_POSITION, Constants.kLongCANTimeoutSec);
    Util.checkReturn(
        "arm position",
        mArmTalon.getPosition().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
    Util.checkReturn(
        "arm current",
        mArmTalon.getTorqueCurrent().setUpdateFrequency(50, Constants.kLongCANTimeoutSec));
    Util.checkReturn("arm canbus", mArmTalon.optimizeBusUtilization(Constants.kLongCANTimeoutSec));

    stop();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(getName() + "Target Angle Degree", () -> getTargetAngleDeg(), null);
    builder.addDoubleProperty(getName() + "Angle Degree", () -> getAngleDeg(), null);
    builder.addDoubleProperty(getName() + "Current", () -> getStatorCurrent(), null);
    builder.addStringProperty(
        getName() + "Active Control Request", () -> mArmTalon.getAppliedControl().toString(), null);
  }

  @Override
  public void periodic() {}

  public void setAngle(double angle_deg) {
    if (angle_deg < ArmConstants.ARM_REST_ANGLE || angle_deg > ArmConstants.ARM_MAX_ANGLE) return;

    setReverseLimit(true);
    double angle_rotation = angle_deg / 360.0;
    mArmTalon.setControl(
        ArmMotionMagic.withPosition(
            angle_rotation)); // reuse previously created ControlRequest Instance
  }

  public void stop() {
    mArmTalon.setControl(Constants.NEUTRAL);
    setReverseLimit(true);
  }

  public void setVoltage(double voltage) {
    mArmTalon.setControl(ArmVoltage.withOutput(voltage));
  }

  public double getRotation() {
    return mArmTalon.getPosition().getValueAsDouble();
  }

  public double getStatorCurrent() {
    return mArmTalon.getStatorCurrent().getValueAsDouble();
  }

  public double getSupplyCurrent() {
    return mArmTalon.getSupplyCurrent().getValueAsDouble();
  }

  public double getTargetAngleDeg() {
    if (mArmTalon.getAppliedControl().getClass() == MotionMagicVoltage.class) {
      return ArmMotionMagic.Position * 360.0; // last applied motion magic value
    } else {
      return Double.NaN;
    }
  }

  public double getAngleDeg() {
    return getRotation() * 360.0;
  }

  public void setReverseLimit(boolean enable) {
    if (mSoftLimitConf.ReverseSoftLimitEnable != enable) {
      mSoftLimitConf.ReverseSoftLimitEnable = enable;
      mArmTalon.getConfigurator().apply(mSoftLimitConf);
    }
  }

  public void setTalonToInitPosition() {
    mArmTalon.setPosition(ArmConstants.ARM_REST_POSITION);
  }
}
