package frc.robot.auto.modes.Dallas;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.auto.AutoRoutineConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DallasAutoDrop53Routine {
  public enum Drop53Strategy {
    NEAR_SIDE,
    FAR_SIDE
  }

  private static class StrategyParams {
    final AutoRoutineConfig.AutoShootingConfig shootConfigFirstNote;
    final PathPlannerPath firstNotePath;
    final Rotation2d firstNoteRotation;
    final PathPlannerPath gotoDrop53Path;
    final PathPlannerPath endChasePath;

    StrategyParams(
        AutoRoutineConfig.AutoShootingConfig shootConfigFirstNote,
        PathPlannerPath firstNotePath,
        Rotation2d firstNoteRotation,
        PathPlannerPath gotoDrop53Path,
        PathPlannerPath endChasePath) {
      this.shootConfigFirstNote = shootConfigFirstNote;
      this.firstNotePath = firstNotePath;
      this.firstNoteRotation = firstNoteRotation;
      this.gotoDrop53Path = gotoDrop53Path;
      this.endChasePath = endChasePath;
    }
  }

  private static StrategyParams getStrategyParams(Drop53Strategy strategy) {
    switch (strategy) {
      case NEAR_SIDE:
        return new StrategyParams(
            AutoRoutineConfig.AutoShootPositions.UNDER_STAGE,
            AutoRoutineConfig.AutoPaths.FROM_53_TO_52,
            Rotation2d.fromDegrees(90),
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_54,
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_55);

      case FAR_SIDE:
        return new StrategyParams(
            AutoRoutineConfig.AutoShootPositions.UNDER_STAGE,
            AutoRoutineConfig.AutoPaths.FROM_53_TO_54,
            Rotation2d.fromDegrees(-90),
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_52,
            AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_55);
      default:
        throw new IllegalArgumentException("Invalid strategy for DallasAutoDrop53Routine");
    }
  }

  public static Command buildDrop53Command(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      Drop53Strategy strategy,
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
        buildIntakeShootWhileMovingCommandWithIntake53(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoRoutineConfig.AutoPaths.START_DALLAS,
            AutoRoutineConfig.AutoShootPositions.NOTE_32.shootParams,
            fallbackRotation53),

        // Drop 53 and Get first note
        drop53ThenBuildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(params.firstNotePath),
            params.firstNoteRotation),

        // Score first note
        AutoBuilder
            .pathfindThenFollowPath(AutoRoutineConfig.AutoPaths.APPROACH_UNDER_STAGE, PathfindConstants.constraints)
            .deadlineWith(
                new IntakeCommand(intake, transfer),
                new SetArmAngleCommand(arm, params.shootConfigFirstNote.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootConfigFirstNote.shootParams.speed_rps)),

        // Go to dropped 53
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(params.gotoDrop53Path),
            null),

        // Score dropped 53
        AutoBuilder
            .pathfindThenFollowPath(AutoRoutineConfig.AutoPaths.APPROACH_UNDER_STAGE, PathfindConstants.constraints)
            .deadlineWith(
                new IntakeCommand(intake, transfer),
                new SetArmAngleCommand(arm, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter,
                    AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.speed_rps)),

        // Move to chase 55
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(params.endChasePath),
            Rotation2d.fromDegrees(90)));
  }

  public static Command buildIntakeShootWhileMovingCommandWithIntake53(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      GamePieceProcessor gamePieceProcessor,
      PathPlannerPath pathName,
      ShootingParameters shootParams,
      Rotation2d findNoteHeading) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(pathName),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetArmAngleCommand(arm, shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, shootParams.speed_rps),
                    new IntakeAndFeedCommand(intake, transfer)),
                new ParallelCommandGroup(
                    new SetShooterTargetCommand(shooter, 0.0),
                    new SetArmAngleCommand(arm, ArmConstants.ARM_OBSERVE_ANGLE),
                    new IntakeCommand(intake, transfer))))
            .until(
                () -> {
                  boolean deadline = AutoCommandFactory.isFieldPositionReached(
                      drivetrainSubsystem, AutoCommandFactory.kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget);
                }),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(
                () -> AutoCommandFactory.isFieldPositionReached(
                    drivetrainSubsystem, AutoCommandFactory.kMidFieldFenceX))
            .until(() -> transfer.isOmronDetected())
            .alongWith(new SetShooterTargetCommand(shooter, 17)),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note")));
  }

  public static Command drop53ThenBuildPathThenChaseNoteCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      GamePieceProcessor gamePieceProcessor,
      Command pathCommand,
      Rotation2d findNoteHeading) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            pathCommand,
            new SequentialCommandGroup(
                new WaitCommand(0.0).andThen(new FeedCommand(transfer, shooter)),
                Commands.runOnce(
                    () -> {
                      shooter.stop();
                      SmartDashboard.putString("Auto Status", "dropped 53");
                    },
                    shooter),
                new ParallelCommandGroup(
                    new SetArmAngleCommand(arm, ArmConstants.ARM_OBSERVE_ANGLE),
                    new IntakeCommand(intake, transfer))))
            .until(
                () -> {
                  boolean deadline = AutoCommandFactory.isFieldPositionReached(
                      drivetrainSubsystem, AutoCommandFactory.kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget && !transfer.isOmronDetected());
                }),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(
                () -> {
                  boolean midbar = AutoCommandFactory.isFieldPositionReached(
                      drivetrainSubsystem, AutoCommandFactory.kMidFieldFenceX);
                  if (midbar)
                    SmartDashboard.putString("Auto Status", "chase interrupt because midfield bar");
                  return midbar;
                })
            .unless(() -> transfer.isOmronDetected()),
        Commands.either(
            new WaitCommand(0),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotating to find note")),
                new TurnToHeadingCommand(drivetrainSubsystem, findNoteHeading)
                    .deadlineWith(new IntakeCommand(intake, transfer))
                    .until(() -> gamePieceProcessor.getClosestGamePieceInfo().isPresent()),
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotation Finished")),
                new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
                    .until(
                        () -> {
                          boolean midbar = AutoCommandFactory.isFieldPositionReached(
                              drivetrainSubsystem, AutoCommandFactory.kMidFieldFenceX);
                          if (midbar)
                            SmartDashboard.putString(
                                "Auto Status", "chase interrupt because midfield bar");
                          return midbar;
                        }))
                .until(() -> transfer.isOmronDetected()),
            () -> {
              Boolean hasNote = transfer.isOmronDetected();
              SmartDashboard.putBoolean("Has Note", hasNote);
              if (hasNote) {
                SmartDashboard.putString("Auto Status", "hasnote=" + hasNote.toString());
              }
              return hasNote;
            }));
  }
}