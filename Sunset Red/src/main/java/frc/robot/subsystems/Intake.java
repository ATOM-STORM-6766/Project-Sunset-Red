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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private static final double INTAKE_VOLTS = 3.0;
  private static final double INTAKE_CENTER_PERC = 0.3;
  private static final double OUTTAKE_VOLTS = -6.0;
  private static final double OUTTAKE_CENTER_PERC = -0.4;

  public enum IntakeState {
    STOP,
    INTAKE,
    OUTTAKE
  }

  private IntakeState mIntakeState = IntakeState.STOP;
  private final TalonFX mIntakeMotor;
  private final VictorSPX mCenterMotor;
  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  private static class PeriodicIO {
    // OUTPUTS
    public VoltageOut ctrlVal = new VoltageOut(0.0);
    public double centerPerc = 0.0;
  }

  public Intake() {
    mIntakeMotor = new TalonFX(IntakeConstants.INTAKER_ID);
    mCenterMotor = new VictorSPX(IntakeConstants.INTAKER_CENTER_ID);

    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
    intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
    mCenterMotor.setNeutralMode(NeutralMode.Brake);
    mCenterMotor.set(ControlMode.PercentOutput, 0.0);

    stop();
  }

  @Override
  public void periodic() {
    mIntakeMotor.setControl(mPeriodicIO.ctrlVal);
    mCenterMotor.set(ControlMode.PercentOutput, mPeriodicIO.centerPerc);
  }

  public synchronized void setIntake() {
    mIntakeState = IntakeState.INTAKE;
    mPeriodicIO.ctrlVal.Output = INTAKE_VOLTS;
    mPeriodicIO.centerPerc = INTAKE_CENTER_PERC;
  }

  public synchronized void setOuttake() {
    mIntakeState = IntakeState.OUTTAKE;
    mPeriodicIO.ctrlVal.Output = OUTTAKE_VOLTS;
    mPeriodicIO.centerPerc = OUTTAKE_CENTER_PERC;
  }

  public synchronized void stop() {
    mIntakeState = IntakeState.STOP;
    mPeriodicIO.ctrlVal.Output = 0.0;
    mPeriodicIO.centerPerc = 0.0;
  }

  public synchronized IntakeState getIntakeState() {
    return mIntakeState;
  }
}
