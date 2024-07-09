package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ChaseNoteStateMachineCommand extends Command {
  private enum State {
    CHASING,
    INTAKING,
    END
  }

  private final DrivetrainSubsystem sDrivetrainSubsystem;
  private final GamePieceProcessor sGamePieceProcessor;
  private final Intake sIntake;
  private final Transfer sTransfer;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private State currentState;

  // Constructor
  public ChaseNoteStateMachineCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      GamePieceProcessor gamePieceProcessor,
      Intake intake,
      Transfer transfer) {
    this.sDrivetrainSubsystem = drivetrainSubsystem;
    this.sGamePieceProcessor = gamePieceProcessor;
    this.sIntake = intake;
    this.sTransfer = transfer;

    xController = new PIDController(0.2, 0.0, 0.0); // Adjust PID values as needed
    yController = new PIDController(0.0, 0.0, 0.0); // Adjust PID values as needed
    rotationController = new PIDController(0.2, 0.0, 0.0); // Adjust PID values as needed

    addRequirements(drivetrainSubsystem, gamePieceProcessor, intake, transfer);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotationController.reset();
    currentState = State.CHASING;
  }

  @Override
  public void execute() {
    switch (currentState) {
      case CHASING:
        // Chase the note
        Optional<PhotonTrackedTarget> targetOptional =
            sGamePieceProcessor.getClosestGamePieceInfo();
        boolean isTargetPresent = targetOptional.isPresent();

        if (isTargetPresent) {
          PhotonTrackedTarget target = targetOptional.get();
          double yawMeasure = target.getYaw();
          double pitchMeasure = target.getPitch();

          if (sIntake.isOmronDetected() || pitchMeasure > 12) {
            currentState = State.INTAKING;
          } else {
            // drive the robot
            double xOutput = xController.calculate(pitchMeasure, 20);
            double yOutput = -yController.calculate(yawMeasure, 0);

            Translation2d driveVector = new Translation2d(xOutput, yOutput);
            double angularVelocity = -rotationController.calculate(yawMeasure, 0);

            sDrivetrainSubsystem.drive(driveVector, angularVelocity, false);
            sIntake.setIntake();
            sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
          }
        } else {
          // End the command because the note is not present
          currentState = State.END;
        }

        break;
      case INTAKING:
        // Intake the note
        sIntake.setIntake();
        sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
        sDrivetrainSubsystem.drive(new Translation2d(2, 0), 0, false);
        if (sTransfer.isOmronDetected()) {
          currentState = State.END;
        }
        break;
      case END:
        // End the command
        sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    sIntake.stop();
    sTransfer.stop();
  }

  @Override
  public boolean isFinished() {
    return currentState == State.END;
  }
}
