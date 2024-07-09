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
import frc.robot.utils.Util;

public class Transfer extends SubsystemBase {

  public static final double INTAKE_VOLTS = 4.0;
  public static final double FEED_VOLTS = 8.0;
  public static final double AMP_RELEASE_VOLTS = 8.0;
  public static final double OUTTAKE_VOLTS = -4.0;

  private final TalonFX mTransferTalon;
  private final DigitalInput mTransferOmron;

  private VoltageOut transferVoltage = new VoltageOut(0.0);

  public Transfer() {
    mTransferTalon = new TalonFX(Constants.TransferConstants.TRANSFER_ID);
    mTransferOmron = new DigitalInput(Constants.TransferConstants.TRANSFER_OMRON_PORT);

    TalonFXConfiguration transferConfigs = new TalonFXConfiguration();
    transferConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    transferConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    transferConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    transferConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    transferConfigs.Voltage.PeakForwardVoltage = 12.0;
    transferConfigs.Voltage.PeakReverseVoltage = -12.0;
    transferConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    transferConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    transferConfigs.CurrentLimits.SupplyCurrentThreshold = 20.0;
    transferConfigs.CurrentLimits.SupplyTimeThreshold = 0.0;
    Util.checkReturn(
        "transfer",
        mTransferTalon.getConfigurator().apply(transferConfigs, Constants.kLongCANTimeoutSec));
    mTransferTalon.setControl(Constants.NEUTRAL);
    Util.checkReturn(
        "transfer canbus", mTransferTalon.optimizeBusUtilization(Constants.kLongCANTimeoutSec));

    stop();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty(
        getName() + "Transfer Motor Control Request",
        () -> mTransferTalon.getAppliedControl().toString(),
        null);
    builder.addBooleanProperty(getName() + "Omron Detected", () -> isOmronDetected(), null);
  }

  @Override
  public void periodic() {}

  public void setVoltage(double voltage) {
    mTransferTalon.setControl(transferVoltage.withOutput(voltage));
  }

  public void stop() {
    mTransferTalon.setControl(Constants.NEUTRAL); // lowest latency
    setVoltage(0.0);
  }

  public boolean isOmronDetected() {
    return !mTransferOmron.get(); // lowest latency
  }
}
