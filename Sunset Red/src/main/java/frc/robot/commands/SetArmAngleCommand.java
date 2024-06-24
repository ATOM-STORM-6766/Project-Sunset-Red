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
  private boolean isFinished;

  private final double ERR_TOL = 1.0; // degree
  private final double LOW_SHOOT_TOLERANCE_ROTATION = 23.5;

  public SetArmAngleCommand(Arm arm, double angle) {
    sArm = arm;
    targetAngle = angle;
    addRequirements(sArm);
    System.out.println("target angle: " + targetAngle);
  }

  @Override
  public void initialize() {
    sArm.setAngle(targetAngle);
    isFinished = false;
    mArmInPosition = new DelayedBoolean(Timer.getFPGATimestamp(), ArmConstants.STABILIZE_TIME);
  }

  @Override
  public void execute() {
    // if going rest and almost reach, brake and let gravity do the job.
    if(targetAngle < ArmConstants.ARM_REST_ANGLE + 3.0 && sArm.getAngleDeg() < ArmConstants.ARM_REST_ANGLE + 5.0){
      sArm.stop(); // Neutral Out
      isFinished = true;
      System.out.println("Arm Homed");
      return;
    }
    
    // if going to shoot
    // TODO: amp & climb?
    if(targetAngle > ArmConstants.ARM_REST_ANGLE + 3.0 && shootErrorTolerated()){
      isFinished = true;
      System.out.println("Arm Target Reached");

      return;
    }
  }

  public boolean shootErrorTolerated(){
    boolean currently_in_position = Math.abs(targetAngle - sArm.getAngleDeg()) < ERR_TOL // 1度以内
    || (targetAngle < LOW_SHOOT_TOLERANCE_ROTATION // ? 
        && sArm.getAngleDeg() < LOW_SHOOT_TOLERANCE_ROTATION); // ?
    return mArmInPosition.update(Timer.getFPGATimestamp(), currently_in_position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
