package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private static final double INTAKE_VOLTS = 3.0;
  private static final double INTAKE_CENTER_PERC = 0.3;
  private static final double OUTTAKE_VOLTS = -3.0;
  private static final double OUTTAKE_CENTER_PERC = -0.3;
  private static final double EXTERIOR_PERC = 0.4;

  private final TalonFX mIntakeMotor;
  private final VictorSPX mCenterMotor;
  private final CANSparkMax mExteriorIntakeMotor;
  private final DigitalInput mIntakeOmron;

  private final VoltageOut intakeVoltage = new VoltageOut(0);

  public Intake() {
    mIntakeMotor = new TalonFX(IntakeConstants.INTAKER_ID);
    mCenterMotor = new VictorSPX(IntakeConstants.INTAKER_CENTER_ID);
    mIntakeOmron = new DigitalInput(IntakeConstants.INTAKER_ENTER_OMRON_ID);
    mExteriorIntakeMotor =
        new CANSparkMax(IntakeConstants.INTAKE_EXTERIOR_ID, MotorType.kBrushless);

    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
    intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    intakeConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    intakeConfigs.Voltage.PeakForwardVoltage = 12.0;
    intakeConfigs.Voltage.PeakReverseVoltage = -12.0;
    intakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    intakeConfigs.CurrentLimits.SupplyCurrentThreshold = 20.0;
    intakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.0;
    mIntakeMotor.getConfigurator().apply(intakeConfigs);
    mIntakeMotor.setControl(Constants.NEUTRAL);
    mIntakeMotor.optimizeBusUtilization();

    VictorSPXConfiguration centerConfigs = new VictorSPXConfiguration();
    centerConfigs.forwardSoftLimitEnable = false;
    centerConfigs.reverseSoftLimitEnable = false;
    centerConfigs.peakOutputForward = 1.0;
    centerConfigs.peakOutputReverse = -1.0;
    mCenterMotor.configAllSettings(centerConfigs);
    mCenterMotor.setInverted(false);
    mCenterMotor.setNeutralMode(NeutralMode.Coast);
    mCenterMotor.set(ControlMode.PercentOutput, 0.0);

    mExteriorIntakeMotor.clearFaults();
    mExteriorIntakeMotor.setInverted(true);
    mExteriorIntakeMotor.setIdleMode(
        IdleMode.kCoast); // do not change to brake, otherwise there will be a lot of heat
    mExteriorIntakeMotor.setVoltage(0);
    mExteriorIntakeMotor.burnFlash();

    stop();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty(
        getName() + "Intake Motor Control Request",
        () -> mIntakeMotor.getAppliedControl().toString(),
        null);
    builder.addDoubleProperty(
        getName() + "Intake Motor Temp",
        () -> mIntakeMotor.getDeviceTemp().getValueAsDouble(),
        null);
    builder.addDoubleProperty(
        getName() + "Center Motor Voltage", () -> mCenterMotor.getMotorOutputVoltage(), null);
  }

  public synchronized void setIntake() {
    mIntakeMotor.setControl(intakeVoltage.withOutput(INTAKE_VOLTS));
    mCenterMotor.set(ControlMode.PercentOutput, INTAKE_CENTER_PERC);
    mExteriorIntakeMotor.set(EXTERIOR_PERC);
  }

  public synchronized void setOuttake() {
    mIntakeMotor.setControl(intakeVoltage.withOutput(OUTTAKE_VOLTS));
    mCenterMotor.set(ControlMode.PercentOutput, OUTTAKE_CENTER_PERC);
    mExteriorIntakeMotor.set(-EXTERIOR_PERC);
  }

  public synchronized void stop() {
    mIntakeMotor.setControl(Constants.NEUTRAL);
    mCenterMotor.set(ControlMode.PercentOutput, 0);
    mExteriorIntakeMotor.set(0.0);
  }

  public boolean isOmronDetected() {
    return !mIntakeOmron.get();
  }
}
