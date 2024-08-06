package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class ChaseNoteCommand extends Command {
  private enum State {
    CHASING,
    INTAKING,
    END
  }

  private final DrivetrainSubsystem sDrivetrainSubsystem;
  private final Intake sIntake;
  private final Transfer sTransfer;
  private final Arm sArm;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  private State currentState;

  private SetArmAngleCommand setArmAngleCommand;
  private Timer stateTimer;

  // constants for the command
  private static final double INTAKE_STATE_TIMEOUT =
      2.0; // INTAKE will be considered failed if it takes longer than
  // this
  private static final double STATE_TIMEOUT =
      5.0; // other states will be considered failed if they take longer than
  // this
  private static final double[] X_CONTROLLER_CONSTANTS = {0.2, 0.0, 0.0};
  private static final double[] Y_CONTROLLER_CONSTANTS = {0.0, 0.0, 0.0};
  private static final double[] ROT_CONTROLLER_CONSTANTS = {0.15, 0.0, 0.0};
  private static final double INTAKE_STATE_DRIVE_SPEED = 1.75; // adjust as needed
  private static final double CLOSE_PITCH_THRESHOLD =
      3; // todo: measure this during field measurement
  private static final double CLOSE_YAW_THRESHOLD =
      5; // todo: measure this during field measurement
  private static final double TARGET_PITCH = 0;

  public ChaseNoteCommand(
      DrivetrainSubsystem sDrivetrainSubsystem, Intake sIntake, Transfer sTransfer, Arm sArm) {
    this.sDrivetrainSubsystem = sDrivetrainSubsystem;
    this.sIntake = sIntake;
    this.sTransfer = sTransfer;
    this.sArm = sArm;

    xController =
        new PIDController(
            X_CONTROLLER_CONSTANTS[0], X_CONTROLLER_CONSTANTS[1], X_CONTROLLER_CONSTANTS[2]);
    yController =
        new PIDController(
            Y_CONTROLLER_CONSTANTS[0], Y_CONTROLLER_CONSTANTS[1], Y_CONTROLLER_CONSTANTS[2]);
    rotController =
        new PIDController(
            ROT_CONTROLLER_CONSTANTS[0], ROT_CONTROLLER_CONSTANTS[1], ROT_CONTROLLER_CONSTANTS[2]);

    this.setArmAngleCommand = new SetArmAngleCommand(sArm, ArmConstants.ARM_OBSERVE_ANGLE);
    stateTimer = new Timer();

    addRequirements(sDrivetrainSubsystem, sIntake, sTransfer, sArm);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();
    currentState = State.CHASING;
    setArmAngleCommand.initialize();
    stateTimer.reset();
    stateTimer.start();
    SmartDashboard.putString("ChaseNote State", "CHASING");
  }

  @Override
  public void execute() {
    setArmAngleCommand.execute(); // we always set the arm angle at intaking pose

    switch (currentState) {
      case CHASING:
        executeChasing();
        break;
      case INTAKING:
        executeIntaking();
        break;
      case END:
        executeEnd();
        break;
    }
    updateDashboard();
  }

  @Override
  public void end(boolean interrupted) {
    sIntake.stop();
    sTransfer.stop();
    sDrivetrainSubsystem.stop();
    setArmAngleCommand.end(interrupted);
    stateTimer.stop();
    SmartDashboard.putString("ChaseNote State", "END (Command Ended)");
    SmartDashboard.putBoolean("ChaseNote Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return currentState == State.END;
  }

  private void executeChasing() {
    var targetOptional = GamePieceProcessor.getInstance().getClosestGamePieceInfo();
    if (targetOptional.isPresent() && !sTransfer.isOmronDetected()) {
      var target = targetOptional.get();
      var pitchMeasure = target.getPitch();
      var yawMeasure = target.getYaw();

      if (isCloseEnoughToNote(pitchMeasure, yawMeasure)) {
        transitionToIntaking();
      } else {
        ChassisSpeeds chaseSpeed = calculateChaseSpeed(pitchMeasure, yawMeasure);
        sDrivetrainSubsystem.driveWithChassisSpeed(chaseSpeed);
        sIntake.setIntake();
        sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
      }
    } else {
      handleLostTarget();
    }
    if (stateTimer.hasElapsed(STATE_TIMEOUT)) {
      transitionToEnd("Chasing Timeout");
    }
  }

  private void executeIntaking() {
    sIntake.setIntake();
    sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
    sDrivetrainSubsystem.drive(
        new Translation2d(INTAKE_STATE_DRIVE_SPEED, 0), 0, false); // in intake state, we
    // only drive forward,
    // assume we are close
    // enough to the note

    if (sTransfer.isOmronDetected()) {
      transitionToEnd("Note Acquired");
    } else if (stateTimer.hasElapsed(INTAKE_STATE_TIMEOUT)) {
      transitionToEnd("Intaking Timeout");
    }
  }

  private void executeEnd() {
    sDrivetrainSubsystem.stop();
    sIntake.stop();
    sTransfer.stop();
  }

  private boolean isCloseEnoughToNote(double pitchMeasure, double yawMeasure) {
    return sIntake.isOmronDetected()
        || (pitchMeasure < CLOSE_PITCH_THRESHOLD && Math.abs(yawMeasure) < CLOSE_YAW_THRESHOLD);
  }

  private ChassisSpeeds calculateChaseSpeed(double pitchMeasure, double yawMeasure) {
    /*
     * todo:
     * the sign of these outputs may need to be adjusted based on the install
     * direction of the camera
     */
    var xOutput = -xController.calculate(pitchMeasure, TARGET_PITCH);
    var yOutput =
        yController.calculate(
            yawMeasure, 0); // camera centered: note is always in the middle of the
    // camera view
    var angularVelocity = rotController.calculate(yawMeasure, 0);
    return new ChassisSpeeds(xOutput, yOutput, angularVelocity);
  }

  private void handleLostTarget() {
    sDrivetrainSubsystem.stop();
    transitionToEnd("Target Lost During Chasing");
  }

  private void transitionToIntaking() {
    currentState = State.INTAKING;
    stateTimer.reset();
    SmartDashboard.putString("ChaseNote State", "INTAKING");
  }

  private void transitionToEnd(String reason) {
    currentState = State.END;
    SmartDashboard.putString("ChaseNote State", "END (" + reason + ")");
  }

  private void updateDashboard() {
    SmartDashboard.putString("ChaseNote State", currentState.toString());
    SmartDashboard.putNumber("Chase Timer", stateTimer.get());
    SmartDashboard.putBoolean(
        "Target Present", GamePieceProcessor.getInstance().getClosestGamePieceInfo().isPresent());
    SmartDashboard.putBoolean("Transfer Omron", sTransfer.isOmronDetected());
    SmartDashboard.putNumber("Arm Angle", sArm.getAngleDeg());
    SmartDashboard.putNumber("Drive X Output", xController.getSetpoint());
    SmartDashboard.putNumber("Drive Y Output", yController.getSetpoint());
    SmartDashboard.putNumber("Angular Velocity", rotController.getSetpoint());
  }
}
