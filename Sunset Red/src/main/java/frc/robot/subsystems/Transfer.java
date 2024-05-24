package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  private static class PeriodicIO {
    // INPUTS
    public boolean omronDetected = false;

    // OUTPUTS
    public VoltageOut ctrlval = new VoltageOut(0.0);
  }

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
  public void periodic() {
    mPeriodicIO.omronDetected = !mTransferOmron.get();
    mTransferTalon.setControl(mPeriodicIO.ctrlval);
    outputTelemetry();
  }

  public void setVoltage(double voltage) {
    mPeriodicIO.ctrlval = new VoltageOut(voltage);
  }

  public void stop() {
    setVoltage(0.0);
  }

  public boolean isOmronDetected() {
    return mPeriodicIO.omronDetected;
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Transfer Volt Out", mPeriodicIO.ctrlval.Output);
    SmartDashboard.putBoolean("Transfer Omron Detected", mPeriodicIO.omronDetected);
  }
}
