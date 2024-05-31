package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Util;

public class Arm extends SubsystemBase {
  private static final double ARM_GEAR_RATIO = 80.0;
  // private static final double ARM_CLIMB_TARGET_DEG = 30.0;
  // private static final double ARM_CLIMB_MAX_DEG = 35.0;
  private static final double ERR_TOL = 1.0 / 360.0;
  private static final double STABILIZE_TIME = 0.1;

  private static final double LOW_SHOOT_TOLERANCE_ROTATION = 23.5 / 360.0;

  private final TalonFX mArmTalon;
  // private boolean climbTargetReached = false;
  private final SoftwareLimitSwitchConfigs mSoftLimitConf = new SoftwareLimitSwitchConfigs();

  private static class PeriodicIO {
    public double armPosition = 0.0;
    public double armCurrent = 0.0;
    public ControlRequest activeCtrlReq = new NeutralOut();
    public MotionMagicVoltage ctrlval = new MotionMagicVoltage(ArmConstants.ARM_REST_POSITION);
    public VoltageOut voltval = new VoltageOut(0.0);
  }

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  private static Arm sInstance;

  public static Arm getInstance() {
    if (sInstance == null) {
      sInstance = new Arm();
    }
    return sInstance;
  }

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

      builder.addBooleanProperty("isShootErrorTolerated", ()->shootErrorTolerated(), null);
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    writePeriodicOutputs();
    outputTelemetry();
  }

  private void readPeriodicInputs() {
    mPeriodicIO.armPosition = mArmTalon.getPosition().getValueAsDouble();
    mPeriodicIO.armCurrent = mArmTalon.getTorqueCurrent().getValueAsDouble();
  }

  private void writePeriodicOutputs(){
    if(mPeriodicIO.ctrlval.Position < ArmConstants.ARM_REST_POSITION + 3.0/360 && mPeriodicIO.armPosition < ArmConstants.ARM_REST_POSITION + 5.0/360) {
      // stop power when resting
      mArmTalon.setControl(Constants.NEUTRAL);
    }else{
      mArmTalon.setControl(mPeriodicIO.activeCtrlReq);
    }
  }

  public void setAngle(double angle_deg) {
    if (angle_deg < ArmConstants.ARM_REST_ANGLE || angle_deg > ArmConstants.ARM_MAX_ANGLE) return;

    setReverseLimit(true);
    double angle_rotation = angle_deg / 360.0;
    mPeriodicIO.ctrlval.Position = angle_rotation;
    mPeriodicIO.activeCtrlReq = mPeriodicIO.ctrlval;
  }

  public void stop() {
    mPeriodicIO.ctrlval.Position = ArmConstants.ARM_REST_POSITION;
    mPeriodicIO.activeCtrlReq = Constants.NEUTRAL;
    setReverseLimit(true);
  }

  public void setVoltage(double voltage) {
    mPeriodicIO.voltval.Output = voltage;
    mPeriodicIO.activeCtrlReq = mPeriodicIO.voltval;
  }

  public double getArmCurrent() {
    return mPeriodicIO.armCurrent;
  }

  public boolean shootErrorTolerated() {
    return Math.abs(mPeriodicIO.ctrlval.Position - mPeriodicIO.armPosition) < ERR_TOL
        || (mPeriodicIO.ctrlval.Position < LOW_SHOOT_TOLERANCE_ROTATION
            && mPeriodicIO.armPosition < LOW_SHOOT_TOLERANCE_ROTATION);
  }

  public double getAngleDeg() {
    return mPeriodicIO.armPosition * 360.0;
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

  private void outputTelemetry() {
    SmartDashboard.putNumber("Arm Target Angle", mPeriodicIO.ctrlval.Position * 360);
    SmartDashboard.putNumber("Arm Angle", getAngleDeg());
    SmartDashboard.putNumber("Arm Current", mPeriodicIO.armCurrent);
    SmartDashboard.putString("Arm Control Req", mArmTalon.getAppliedControl().toString());
  }
}
