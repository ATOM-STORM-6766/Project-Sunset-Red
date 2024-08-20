package frc.robot.auto.modes.Arizona;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.auto.AutoRoutineConfig;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class ArizonaAutoSweep {
  /**
   * Arizona Sweep has two modes, NEAR and FAR NEAR mode takes the START_ARIZONA_NEAR path FAR mode
   * takes the START_ARIZONA_FAR path
   *
   * <p>Both modes start with a buildPathThenChaseNoteCommand to intake 51/55 And then they eject
   * note 51/55 using Drop Finally the just directly move to 55/51 (Opposite of the field), in a
   * straight line, to effectively sweep the mid field
   */
  public enum ArizonaSweepMode {
    NEAR,
    FAR
  }

  public static Command buildArizonaSweepCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      ArizonaSweepMode mode) {

    switch (mode) {
      case NEAR:
        return new SequentialCommandGroup(
            AutoCommandFactory.buildPrepCommand(
                drivetrainSubsystem,
                ShootingParameters.BELOW_SPEAKER,
                AutoRoutineConfig.AutoPaths.START_ARIZONA_NEAR,
                arm,
                shooter,
                transfer),
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                    drivetrainSubsystem,
                    arm,
                    shooter,
                    transfer,
                    intake,
                    GamePieceProcessor.getInstance(),
                    AutoBuilder.followPath(AutoRoutineConfig.AutoPaths.START_ARIZONA_NEAR),
                    Rotation2d.fromDegrees(-90))
                .alongWith(new SetShooterTargetCommand(shooter, 17)), // warmup
            AutoBuilder.pathfindToPoseFlipped(
                    FieldConstants.NOTE_55_POSITION, PathfindConstants.constraints)
                .alongWith(new FeedCommand(transfer, shooter)));

      case FAR:
        return new SequentialCommandGroup(
            AutoCommandFactory.buildPrepCommand(
                drivetrainSubsystem,
                ShootingParameters.BELOW_SPEAKER,
                AutoRoutineConfig.AutoPaths.START_ARIZONA_FAR,
                arm,
                shooter,
                transfer),
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                    drivetrainSubsystem,
                    arm,
                    shooter,
                    transfer,
                    intake,
                    GamePieceProcessor.getInstance(),
                    AutoBuilder.followPath(AutoRoutineConfig.AutoPaths.START_ARIZONA_FAR),
                    Rotation2d.fromDegrees(90))
                .alongWith(new SetShooterTargetCommand(shooter, 17)), // warmup
            AutoBuilder.pathfindToPoseFlipped(
                    FieldConstants.NOTE_51_POSITION, PathfindConstants.constraints)
                .alongWith(new FeedCommand(transfer, shooter)));

      default:
        // throw an error
        throw new IllegalArgumentException("Invalid ArizonaSweepMode");
    }
  }
}
