package frc.robot.auto.modes.Dallas;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.auto.AutoRoutineConfig;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class DallasAutoScore53Routine {

  public enum Score53Strategy {
    NEAR,
    FAR
  }

  private static class StrategyParams {
    final AutoRoutineConfig.AutoShootingConfig shootConfigFirstMidNote;
    final AutoRoutineConfig.AutoShootingConfig shootConfigSecondMidNote;
    final PathPlannerPath firstNotePath;
    final PathPlannerPath secondNotePath;
    final Rotation2d firstNoteRotation;
    final Rotation2d secondNoteRotation;
    final Pose2d endChasePose;

    StrategyParams(
        AutoRoutineConfig.AutoShootingConfig shootConfigFirstNote,
        AutoRoutineConfig.AutoShootingConfig shootConfigSecondNote,
        PathPlannerPath firstNotePath,
        PathPlannerPath secondNotePath,
        Rotation2d firstNoteRotation,
        Rotation2d secondNoteRotation,
        Pose2d endChasePose) {
      this.shootConfigFirstMidNote = shootConfigFirstNote;
      this.shootConfigSecondMidNote = shootConfigSecondNote;
      this.firstNotePath = firstNotePath;
      this.secondNotePath = secondNotePath;
      this.firstNoteRotation = firstNoteRotation;
      this.secondNoteRotation = secondNoteRotation;
      this.endChasePose = endChasePose;
    }
  }

  private static StrategyParams getStrategyParams(Score53Strategy strategy) {
    switch (strategy) {
      case NEAR:
        return new StrategyParams(
            AutoRoutineConfig.AutoShootPositions.UNDER_STAGE,
            AutoRoutineConfig.AutoShootPositions.NEAR_SIDE,
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_52,
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_51,
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(-90),
            FieldConstants.NOTE_55_POSITION);
      case FAR:
        return new StrategyParams(
            AutoRoutineConfig.AutoShootPositions.UNDER_STAGE,
            AutoRoutineConfig.AutoShootPositions.FAR_SIDE,
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_54,
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_55,
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(90),
            FieldConstants.NOTE_55_POSITION);
      default:
        throw new IllegalArgumentException("Invalid strategy for DallasAutoScore53Routine");
    }
  }

  public static Command buildScore53Command(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      Score53Strategy strategy,
      Rotation2d fallbackRotation53) {
    StrategyParams params = getStrategyParams(strategy);

    return new SequentialCommandGroup(
        // Prepare routine
        AutoCommandFactory.buildPrepCommand(
            drivetrainSubsystem,
            ShootingParameters.BELOW_SPEAKER,
            AutoRoutineConfig.AutoPaths.START_DALLAS,
            arm,
            shooter,
            transfer),

        // Move to 53, intake and shoot 32 along the way
        AutoCommandFactory.buildIntakeShootWhileMovingCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoRoutineConfig.AutoPaths.START_DALLAS,
            AutoRoutineConfig.AutoShootPositions.NOTE_32.shootParams,
            fallbackRotation53),

        // Shoot 53
        AutoBuilder.pathfindThenFollowPath(AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.approachShootPosePath, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.speed_rps)),

        // Get first note
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(params.firstNotePath),
            params.firstNoteRotation),

        // Score first note
        AutoBuilder.pathfindThenFollowPath(params.shootConfigFirstMidNote.approachShootPosePath, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, params.shootConfigFirstMidNote.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootConfigFirstMidNote.shootParams.speed_rps)),

        // Get second note
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(params.secondNotePath),
            params.secondNoteRotation),

        // Score second note
        AutoBuilder.pathfindThenFollowPath(params.shootConfigSecondMidNote.approachShootPosePath, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, params.shootConfigSecondMidNote.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootConfigSecondMidNote.shootParams.speed_rps)),

        // End chase
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.pathfindToPoseFlipped(params.endChasePose, PathfindConstants.constraints),
            Rotation2d.fromDegrees(90)));
  }
}