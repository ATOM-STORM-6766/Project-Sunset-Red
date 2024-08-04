package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Arm;
import java.util.Map;
/**
 * @brief This command sets a single target (which must be determined at code initilization/before
 *     match)
 */
public class SetArmAngleCommand extends Command {
  private final Arm sArm;
  private final Double targetAngle;
  private DualEdgeDelayedBoolean mArmInPosition;
  private boolean isFinished;
  private static GenericEntry armAngleEntry= Shuffleboard.getTab("SmartDashboard").add("angle",30).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",30,"max",60)).getEntry();;
  private final double ERR_TOL = 1.0; // degree

  public SetArmAngleCommand(Arm arm, Double targetAngleDegree) {
    assert targetAngleDegree > ArmConstants.ARM_REST_ANGLE
        && targetAngleDegree < ArmConstants.ARM_MAX_ANGLE;
    sArm = arm;
    targetAngle = targetAngleDegree;
    addRequirements(sArm);
  }

  @Override
  public void initialize() {

    sArm.setAngle(armAngleEntry.getDouble(30));
    isFinished = false;
    mArmInPosition =
        new DualEdgeDelayedBoolean(
            Timer.getFPGATimestamp(), ArmConstants.STABILIZE_TIME, EdgeType.RISING);
  }

  @Override
  public void execute() {
    // if going rest and almost reach, brake and let gravity do the job.
    /*
     * This part of logic is here because it needs to be run in loop If it is put in subsystem, then
     * we must introduce states in subsystem, which is not desired.
     */
    if (armAngleEntry.getDouble(30) < ArmConstants.ARM_REST_ANGLE + 3.0
        && sArm.getAngleDeg() < ArmConstants.ARM_REST_ANGLE + 5.0) {
      sArm.stop(); // Neutral Out
      isFinished = true;
      return;
    }

    // if going to shoot
    if (armAngleEntry.getDouble(30) > ArmConstants.ARM_REST_ANGLE + 3.0 && shootErrorTolerated()) {
      isFinished = true;
      return;
    }
  }

  public boolean shootErrorTolerated() {
    boolean currently_in_position = Math.abs(armAngleEntry.getDouble(30)- sArm.getAngleDeg()) < ERR_TOL; // 1度以内
    return mArmInPosition.update(Timer.getFPGATimestamp(), currently_in_position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
