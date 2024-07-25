package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private static final double INTAKE_VOLTS = 3.0;
  private static final double OUTTAKE_VOLTS = -3.0;
  private static final double EXTERNAL_ROLLER_VOLTS = -5.0;

  private final TalonFX mIntakeMotor;
  private final TalonFX mExternalRoller; // New motor
  private final DigitalInput mIntakeOmron;

  private final VoltageOut intakeVoltage = new VoltageOut(0);
  private final VoltageOut externalRollerVoltage = new VoltageOut(0); // New VoltageOut for external roller

  public Intake() {
    mIntakeMotor = new TalonFX(IntakeConstants.INTAKER_ID);
    mIntakeOmron = new DigitalInput(IntakeConstants.INTAKER_ENTER_OMRON_ID);
    mExternalRoller = new TalonFX(IntakeConstants.EXTERNAL_ROLLER_ID); // Initialize new motor

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

    // Configure mExternalRoller (similar to mIntakeMotor)
    mExternalRoller.getConfigurator().apply(intakeConfigs);
    mExternalRoller.setControl(Constants.NEUTRAL);
    mExternalRoller.optimizeBusUtilization();

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
    // Add external roller to sendable
    builder.addStringProperty(
        getName() + "External Roller Control Request",
        () -> mExternalRoller.getAppliedControl().toString(),
        null);
    builder.addDoubleProperty(
        getName() + "External Roller Temp",
        () -> mExternalRoller.getDeviceTemp().getValueAsDouble(),
        null);
  }

  public synchronized void setIntake() {
    mIntakeMotor.setControl(intakeVoltage.withOutput(INTAKE_VOLTS));
    mExternalRoller.setControl(externalRollerVoltage.withOutput(EXTERNAL_ROLLER_VOLTS));
  }

  public synchronized void setOuttake() {
    mIntakeMotor.setControl(intakeVoltage.withOutput(OUTTAKE_VOLTS));
    mExternalRoller.setControl(externalRollerVoltage.withOutput(-EXTERNAL_ROLLER_VOLTS));
  }

  public synchronized void stop() {
    mIntakeMotor.setControl(Constants.NEUTRAL);
    mExternalRoller.setControl(Constants.NEUTRAL);
  }

  public boolean isOmronDetected() {
    return !mIntakeOmron.get();
  }
}
