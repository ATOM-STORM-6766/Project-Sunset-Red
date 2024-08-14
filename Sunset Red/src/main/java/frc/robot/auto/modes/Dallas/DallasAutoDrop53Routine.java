package frc.robot.auto.modes.Dallas;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
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
    final AutoRoutineConfig.AutoShootingConfig shootConfigSecondNote;
    final String firstNotePath;
    final String secondNotePath;
    final Rotation2d firstNoteRotation;
    final Rotation2d secondNoteRotation;
    final String gotoDrop53Path;
    final Pose2d endChasePose;

    StrategyParams(
        AutoRoutineConfig.AutoShootingConfig shootConfigFirstNote,
        AutoRoutineConfig.AutoShootingConfig shootConfigSecondNote,
        String firstNotePath,
        String secondNotePath,
        Rotation2d firstNoteRotation,
        Rotation2d secondNoteRotation,
        String gotoDrop53Path,
        Pose2d endChasePose) {
      this.shootConfigFirstNote = shootConfigFirstNote;
      this.shootConfigSecondNote = shootConfigSecondNote;
      this.firstNotePath = firstNotePath;
      this.secondNotePath = secondNotePath;
      this.firstNoteRotation = firstNoteRotation;
      this.secondNoteRotation = secondNoteRotation;
      this.gotoDrop53Path = gotoDrop53Path;
      this.endChasePose = endChasePose;
    }
  }

  private static StrategyParams getStrategyParams(Drop53Strategy strategy) {
    switch (strategy) {
      case NEAR_SIDE:
        return new StrategyParams(
            AutoRoutineConfig.AutoShootPositions.NEAR_SIDE,
            AutoRoutineConfig.AutoShootPositions.NEAR_SIDE,
            AutoRoutineConfig.AutoPaths.FROM_53_TO_52,
            AutoRoutineConfig.AutoPaths.NEAR_SIDE_TO_51,
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(-90),
            AutoRoutineConfig.AutoPaths.NEAR_SIDE_TO_53,
            FieldConstants.NOTE_54_POSITION);

      case FAR_SIDE:
        return new StrategyParams(
            AutoRoutineConfig.AutoShootPositions.FAR_SIDE,
            AutoRoutineConfig.AutoShootPositions.FAR_SIDE,
            AutoRoutineConfig.AutoPaths.FROM_53_TO_54,
            AutoRoutineConfig.AutoPaths.FAR_SIDE_TO_55,
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(90),
            AutoRoutineConfig.AutoPaths.FAR_SIDE_TO_53,
            FieldConstants.NOTE_55_POSITION);
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
        Commands.either(
            new TurnToHeadingCommand(drivetrainSubsystem, new Rotation2d()),
            new TurnToHeadingCommand(drivetrainSubsystem, Rotation2d.fromDegrees(180)),
            () ->
                DriverStation.getAlliance().isEmpty()
                    || DriverStation.getAlliance().get() == Alliance.Blue),

        // Drop 53 and Get first note
        drop53ThenBuildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.firstNotePath)),
            params.firstNoteRotation),

        // Score first note
        AutoBuilder.pathfindToPoseFlipped(params.shootConfigFirstNote.shootPose, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, params.shootConfigFirstNote.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootConfigFirstNote.shootParams.speed_rps)),

        // Get second note
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.secondNotePath)),
            null),

        // Score second note
        AutoBuilder.pathfindToPoseFlipped(params.shootConfigSecondNote.shootPose, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, params.shootConfigSecondNote.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, params.shootConfigSecondNote.shootParams.speed_rps)),

        // Go to dropped 53
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.gotoDrop53Path)),
            null),

        // Score 53
        AutoBuilder.pathfindToPoseFlipped(AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootPose, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.angle_deg),
                new SetShooterTargetCommand(shooter, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.speed_rps)),

        // Move to midline
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

  public static Command buildIntakeShootWhileMovingCommandWithIntake53(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      GamePieceProcessor gamePieceProcessor,
      String pathName,
      ShootingParameters shootParams,
      Rotation2d findNoteHeading) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)),
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
                  boolean deadline =
                      AutoCommandFactory.isFieldPositionReached(
                          drivetrainSubsystem, AutoCommandFactory.kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target =
                      gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget);
                }),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(
                () ->
                    AutoCommandFactory.isFieldPositionReached(
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
                  boolean deadline =
                      AutoCommandFactory.isFieldPositionReached(
                          drivetrainSubsystem, AutoCommandFactory.kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target =
                      gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget && !transfer.isOmronDetected());
                }),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(
                () -> {
                  boolean midbar =
                      AutoCommandFactory.isFieldPositionReached(
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
                              boolean midbar =
                                  AutoCommandFactory.isFieldPositionReached(
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