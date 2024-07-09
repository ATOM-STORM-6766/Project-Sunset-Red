package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Util;

public class Shooter extends SubsystemBase {

  private final TalonFX mShooterTalon;
  private final TalonFX mShooterFollower;

  private final VelocityVoltage shooterTargetVelocity = new VelocityVoltage(0);

  private DoubleLogEntry mVelocityLog;
  private DoubleLogEntry mTargetVelocityLog;

  public Shooter() {
    mShooterTalon = new TalonFX(ShooterConstants.SHOOTER_ID);
    mShooterFollower = new TalonFX(ShooterConstants.SHOOTER_FOLLOWER);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    shooterConfig.Voltage.PeakForwardVoltage = 12.0;
    shooterConfig.Voltage.PeakReverseVoltage = -12.0;
    shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    shooterConfig.Slot0.kV = 0.113;
    shooterConfig.Slot0.kP = 0.1;
    shooterConfig.Slot0.kI = 1.0;
    shooterConfig.Slot0.kD = 0.0;
    Util.checkReturn(
        "shooter",
        mShooterTalon.getConfigurator().apply(shooterConfig, Constants.kLongCANTimeoutSec));
    shooterConfig.Slot0.kV = 0.118;
    shooterConfig.Slot0.kP = 0.1;
    shooterConfig.Slot0.kI = 1.0;
    shooterConfig.Slot0.kD = 0.0;
    Util.checkReturn(
        "shooter follower",
        mShooterFollower.getConfigurator().apply(shooterConfig, Constants.kLongCANTimeoutSec));
    mShooterTalon.setControl(Constants.NEUTRAL);
    mShooterFollower.setControl(Constants.NEUTRAL);
    Util.checkReturn(
        "shooter velocity",
        mShooterTalon.getVelocity().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
    Util.checkReturn(
        "shooter follower velocity",
        mShooterFollower.getVelocity().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
    Util.checkReturn(
        "shooter voltage",
        mShooterTalon.getMotorVoltage().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
    Util.checkReturn(
        "shooter canbus", mShooterTalon.optimizeBusUtilization(Constants.kLongCANTimeoutSec));
    Util.checkReturn(
        "shooter follower canbus",
        mShooterFollower.optimizeBusUtilization(Constants.kLongCANTimeoutSec));

    initLogEntry();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty(
        getName() + "Control Request", () -> mShooterTalon.getAppliedControl().toString(), null);
    builder.addDoubleProperty("Shooter Main Velocity", () -> getMainMotorVelocity(), null);
    builder.addDoubleProperty("Shooter Follower Velocity", () -> getFollowerVelocity(), null);
    builder.addDoubleProperty("Shooter Target RPS", () -> shooterTargetVelocity.Velocity, null);
  }

  /** */
  private void initLogEntry() {
    String name = getName();

    DataLog log = DataLogManager.getLog();

    mVelocityLog = new DoubleLogEntry(log, name + "/Velocity");
    mTargetVelocityLog = new DoubleLogEntry(log, name + "/TargetVelocity");
  }

  /** @brief add new information to logger */
  private void updateLogEntry() {
    mVelocityLog.append(getAverageVelocity());
    mTargetVelocityLog.append(shooterTargetVelocity.Velocity);
  }

  /**
   * @brief Set the target velocity in rps of the shooter. The shooter will kepp this target until
   *     next call.
   * @param target_rps target velocity in rotations per second
   */
  public void setTargetVelocity(double target_rps) {
    shooterTargetVelocity.Velocity = target_rps;
    mShooterTalon.setControl(shooterTargetVelocity);
    mShooterFollower.setControl(shooterTargetVelocity);
  }

  /** @brief set shooter motor to neutral output */
  public void stop() {
    mShooterTalon.setControl(Constants.NEUTRAL);
    mShooterFollower.setControl(Constants.NEUTRAL);
  }

  public double getMainMotorVelocity() {
    return mShooterTalon.getVelocity().getValueAsDouble();
  }

  public double getFollowerVelocity() {
    return mShooterFollower.getVelocity().getValueAsDouble();
  }

  public double getAverageVelocity() {
    return (getMainMotorVelocity() + getFollowerVelocity()) / 2.0;
  }

  public double getTargetVelocity() {
    return shooterTargetVelocity.Velocity;
  }

  @Override
  public void periodic() {
    updateLogEntry();
  }
}
