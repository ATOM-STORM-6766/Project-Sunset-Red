package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib6907.DelayedBoolean;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ChaseNoteCommand extends ParallelRaceGroup {
  public ChaseNoteCommand(
      DrivetrainSubsystem sDrivetrainSubsystem,
      GamePieceProcessor sGamePieceProcessor,
      Intake sIntake,
      Transfer sTransfer) {
    addCommands(
        new IntakeCommand(sIntake, sTransfer),
        new ChaseCommand(sDrivetrainSubsystem, sGamePieceProcessor).andThen(new WaitCommand(1.0)));
  }

  private class ChaseCommand extends Command {
    private final DrivetrainSubsystem sDrivetrainSubsystem;
    private final GamePieceProcessor sGamePieceCoprocessor;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private DelayedBoolean delayedDetection;
    private static final double DETECTION_DELAY = 0.5; // 100ms delay, adjust as needed

    public ChaseCommand(
        DrivetrainSubsystem drivetrainSubsystem, GamePieceProcessor gamePieceProcessor) {
      this.sDrivetrainSubsystem = drivetrainSubsystem;
      this.sGamePieceCoprocessor = gamePieceProcessor;

      xController = new PIDController(0.2, 0.0, 0.0); // Adjust PID values as needed
      yController = new PIDController(0.0, 0.0, 0.0); // Adjust PID values as needed
      rotationController = new PIDController(0.1, 0.0, 0.0); // Adjust PID values as needed

      delayedDetection = new DelayedBoolean(Timer.getFPGATimestamp(), DETECTION_DELAY);

      addRequirements(drivetrainSubsystem, gamePieceProcessor);
    }

    @Override
    public void initialize() {
      xController.reset();
      yController.reset();
      rotationController.reset();
      delayedDetection = new DelayedBoolean(Timer.getFPGATimestamp(), DETECTION_DELAY);
    }

    @Override
    public void execute() {
      Optional<PhotonTrackedTarget> targetOptional =
          sGamePieceCoprocessor.getClosestGamePieceInfo();
      boolean isTargetPresent = targetOptional.isPresent();
      delayedDetection.update(DETECTION_DELAY, isTargetPresent);

      if (isTargetPresent) {
        PhotonTrackedTarget target = targetOptional.get();

        double yawMeasure = target.getYaw();
        double pitchMeasure = target.getPitch();

        double xOutput = -xController.calculate(pitchMeasure, -22);
        double yOutput = yController.calculate(yawMeasure, 0);

        Translation2d driveVector = new Translation2d(xOutput, yOutput);

        // Calculate rotation based on game piece position
        double angularVelocity = rotationController.calculate(yawMeasure, 0);

        sDrivetrainSubsystem.drive(driveVector, angularVelocity, false);
      } else {
        sDrivetrainSubsystem.drive(new Translation2d(1.0, 0), 0, false);
      }
    }

    @Override
    public void end(boolean interrupted) {
      sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
    }

    @Override
    public boolean isFinished() {
      return !delayedDetection.get();
    }
  }
}
