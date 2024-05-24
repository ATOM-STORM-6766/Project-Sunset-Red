package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.lib6907.DelayedBoolean;
import frc.robot.subsystems.Arm;

public class SetArmAngleCommand extends Command {
  private final Arm sArm;
  private final double targetAngle;
  private DelayedBoolean mArmInPosition;

  public SetArmAngleCommand(Arm arm, double angle) {
    sArm = arm;
    targetAngle = angle;
    addRequirements(sArm);
  }

  @Override
  public void initialize() {
    sArm.setAngle(targetAngle);
    mArmInPosition = new DelayedBoolean(Timer.getFPGATimestamp(), ArmConstants.STABILIZE_TIME);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return mArmInPosition.update(Timer.getFPGATimestamp(), sArm.shootErrorTolerated());
  }
}
