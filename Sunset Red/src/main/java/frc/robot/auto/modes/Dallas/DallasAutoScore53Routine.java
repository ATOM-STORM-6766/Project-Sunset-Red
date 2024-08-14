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
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class DallasAutoScore53Routine {

  public enum Score53Strategy {
    NEAR_SIDE_MID_TO_OUTER,
    NEAR_SIDE_OUTER_TO_MID,
    FAR_SIDE_MID_TO_OUTER,
    FAR_SIDE_OUTER_TO_MID
  }

  private static final String kStartPathDallas = "Dallas StartPath";

  // Shoot poses
  private static final Pose2d kShootPoseUnderStage =
      new Pose2d(4.5, 4.63, Rotation2d.fromDegrees(-14.0));
  private static final Pose2d kShootPoseNearSide =
      new Pose2d(4.5, 6.46, Rotation2d.fromDegrees(14.0));
  private static final Pose2d kShootPoseFarSide =
      new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(28.0));

  // Shooting parameters
  private static final ShootingParameters kShootParamUnderStage = new ShootingParameters(75, 32.5);
  private static final ShootingParameters kShootParamNearSide = new ShootingParameters(75, 32.5);
  private static final ShootingParameters kShootParamFarSide = new ShootingParameters(75, 41);
  private static final ShootingParameters kShootParam32 = new ShootingParameters(75, 36.5);

  // Paths
  private static final String kShootPoseUnderStageTo51Path = "ShootPoseUnderStage to 51";
  private static final String kShootPoseUnderStageTo52Path = "ShootPoseUnderStage to 52";
  private static final String kShootPoseUnderStageTo54Path = "ShootPoseUnderStage to 54";
  private static final String kShootPoseUnderStageTo55Path = "ShootPoseUnderStage to 55";
  private static final String kShootPoseNearSideTo51Path = "ShootPoseNearSide to 51";
  private static final String kShootPoseNearSideTo52Path = "ShootPoseNearSide to 52";
  private static final String kShootPoseFarSideTo54Path = "ShootPoseFarSide to 54";
  private static final String kShootPoseFarSideTo55Path = "ShootPoseFarSide to 55";

  private static class StrategyParams {
    final Pose2d shootPoseFirstNote;
    final Pose2d shootPoseSecondNote;
    final ShootingParameters shootParamsFirstNote;
    final ShootingParameters shootParamsSecondNote;
    final String firstNotePath;
    final String secondNotePath;
    final Rotation2d firstNoteRotation;
    final Rotation2d secondNoteRotation;
    final Pose2d endChasePose;

    StrategyParams(
        Pose2d shootPoseFirstNote,
        Pose2d shootPoseSecondNote,
        ShootingParameters shootParamsFirstNote,
        ShootingParameters shootParamsSecondNote,
        String firstNotePath,
        String secondNotePath,
        Rotation2d firstNoteRotation,
        Rotation2d secondNoteRotation,
        Pose2d endChasePose) {
      this.shootPoseFirstNote = shootPoseFirstNote;
      this.shootPoseSecondNote = shootPoseSecondNote;
      this.shootParamsFirstNote = shootParamsFirstNote;
      this.shootParamsSecondNote = shootParamsSecondNote;
      this.firstNotePath = firstNotePath;
      this.secondNotePath = secondNotePath;
      this.firstNoteRotation = firstNoteRotation;
      this.secondNoteRotation = secondNoteRotation;
      this.endChasePose = endChasePose;
    }
  }

  private static StrategyParams getStrategyParams(Score53Strategy strategy) {
    switch (strategy) {
      case NEAR_SIDE_MID_TO_OUTER:
        return new StrategyParams(
            kShootPoseNearSide,
            kShootPoseNearSide,
            kShootParamNearSide,
            kShootParamNearSide,
            kShootPoseUnderStageTo52Path,
            kShootPoseNearSideTo51Path,
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(-90),
            FieldConstants.NOTE_54_POSITION);
      case NEAR_SIDE_OUTER_TO_MID:
        return new StrategyParams(
            kShootPoseNearSide,
            kShootPoseNearSide,
            kShootParamNearSide,
            kShootParamNearSide,
            kShootPoseUnderStageTo51Path,
            kShootPoseNearSideTo52Path,
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(90),
            FieldConstants.NOTE_54_POSITION);
      case FAR_SIDE_MID_TO_OUTER:
        return new StrategyParams(
            kShootPoseFarSide,
            kShootPoseFarSide,
            kShootParamFarSide,
            kShootParamFarSide,
            kShootPoseUnderStageTo54Path,
            kShootPoseFarSideTo55Path,
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(90),
            FieldConstants.NOTE_55_POSITION);
      case FAR_SIDE_OUTER_TO_MID:
        return new StrategyParams(
            kShootPoseFarSide,
            kShootPoseFarSide,
            kShootParamFarSide,
            kShootParamFarSide,
            kShootPoseUnderStageTo55Path,
            kShootPoseFarSideTo54Path,
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(-90),
            FieldConstants.NOTE_55_POSITION);
      default:
        throw new IllegalArgumentException("Invalid strategy");
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
            kStartPathDallas,
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
            kStartPathDallas,
            kShootParam32,
            fallbackRotation53),

        // Shoot 53
        AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)),

        // Get first note
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.firstNotePath)),
            params.firstNoteRotation),

        // Score first note
        AutoBuilder.pathfindToPoseFlipped(params.shootPoseFirstNote, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, params.shootParamsFirstNote.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootParamsFirstNote.speed_rps)),

        // Get second note
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.secondNotePath)),
            params.secondNoteRotation),

        // Score second note
        AutoBuilder.pathfindToPoseFlipped(params.shootPoseSecondNote, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, params.shootParamsSecondNote.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootParamsSecondNote.speed_rps)),

        // End chase
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.pathfindToPoseFlipped(params.endChasePose, PathfindConstants.constraints),
            Rotation2d.fromDegrees(0)));
  }
}
