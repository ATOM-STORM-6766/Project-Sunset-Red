package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.lib6907.DelayedBoolean;
import frc.robot.subsystems.Arm;

public class InitializeArmCommand extends Command {
  private final Arm mArm;
  private static final double ARM_INIT_VOLTS = -0.5;
  private static final double INIT_CURRENT_THRESHOLD = 10.0;
  private boolean isFinished = false;

  private DelayedBoolean mArmIsInPosition;

  public InitializeArmCommand(Arm arm) {
    mArm = arm;
    addRequirements(mArm);
  }

  @Override
  public void initialize() {
    mArmIsInPosition = new DelayedBoolean(Timer.getFPGATimestamp(), ArmConstants.STABILIZE_TIME);
    isFinished = false;
    mArm.setReverseLimit(false);
    mArm.setVoltage(ARM_INIT_VOLTS);
  }

  @Override
  public void execute() {
    if (mArmIsInPosition.update(
        Timer.getFPGATimestamp(), mArm.getStatorCurrent() < -INIT_CURRENT_THRESHOLD)) {
      mArm.setTalonToInitPosition();
      isFinished = true;
      mArm.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    mArm.stop();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
