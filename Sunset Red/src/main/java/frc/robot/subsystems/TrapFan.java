package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FanConstants;

public class TrapFan extends SubsystemBase {
  private final VictorSPX mFanMotor;

  public TrapFan() {
    mFanMotor = new VictorSPX(FanConstants.FAN_ID);
  }

  public void setFanSpeed(double speed) {
    mFanMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopFan() {
    mFanMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command blowTrapCommand() {
    return this.startEnd(() -> this.setFanSpeed(1.0), () -> this.stopFan());
  }
}
