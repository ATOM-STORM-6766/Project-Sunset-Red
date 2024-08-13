package frc.robot.auto.modes.Dallas;

import java.sql.Driver;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

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
import frc.robot.commands.ChaseNoteCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeAndFeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

/**
 * For Dallas Auto Drop 53 Routine, we have two strategies: NEAR_SIDE and FAR_SIDE For both
 * strategies, we have the following: - Start at Dallas mid position - Move to 53, shoot 32 along
 * the way - On the spot of 53, slowly eject 53 (drop 53) - Move to 52/54 depending on the strategy
 * - Score 52/54 - Move to 51/55 depending on the strategy - Score 51/55 - Come back to 53 and pick
 * up the dropped 53 - Score 53 - Move to midline, for now just put 54 Pose
 */
public class DallasAutoDrop53Routine {
    public enum Drop53Strategy {
        NEAR_SIDE, FAR_SIDE
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
    private static final ShootingParameters kShootParamUnderStage =
            new ShootingParameters(75, 32.5);
    private static final ShootingParameters kShootParamNearSide = new ShootingParameters(75, 32.5);
    private static final ShootingParameters kShootParamFarSide = new ShootingParameters(75, 41);
    private static final ShootingParameters kShootParam32 = new ShootingParameters(75, 32);
    private static final ShootingParameters kShootParamDrop53 = new ShootingParameters(10, 36.5);

    // Paths
    private static final String k53to52Path = "Dallas 53 to 52";
    private static final String k53to54Path = "Dallas 53 to 54";
    private static final String kShootPoseNearSideTo51Path = "ShootPoseNearSide to 51";
    private static final String kShootPoseFarSideTo55Path = "ShootPoseFarSide to 55";
    private static final String kShootPoseNearSideTo53Path = "ShootPoseNearSide to 53";
    private static final String kShootPoseFarSideTo53Path = "ShootPoseFarSide to 53";

    private static class StrategyParams {
        final Pose2d shootPoseFirstNote;
        final Pose2d shootPoseSecondNote;
        final ShootingParameters shootParamsFirstNote;
        final ShootingParameters shootParamsSecondNote;
        final String firstNotePath;
        final String secondNotePath;
        final Rotation2d firstNoteRotation;
        final Rotation2d secondNoteRotation;
        final String gotoDrop53Path;
        final Pose2d endChasePose;

        StrategyParams(Pose2d shootPoseFirstNote, Pose2d shootPoseSecondNote,
                ShootingParameters shootParamsFirstNote, ShootingParameters shootParamsSecondNote,
                String firstNotePath, String secondNotePath, Rotation2d firstNoteRotation,
                Rotation2d secondNoteRotation, String gotoDrop53Path, Pose2d endChasePose) {
            this.shootPoseFirstNote = shootPoseFirstNote;
            this.shootPoseSecondNote = shootPoseSecondNote;
            this.shootParamsFirstNote = shootParamsFirstNote;
            this.shootParamsSecondNote = shootParamsSecondNote;
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
                return new StrategyParams(kShootPoseNearSide, kShootPoseNearSide,
                        kShootParamNearSide, kShootParamNearSide, k53to52Path,
                        kShootPoseNearSideTo51Path, Rotation2d.fromDegrees(90),
                        Rotation2d.fromDegrees(-90), kShootPoseNearSideTo53Path,
                        FieldConstants.NOTE_54_POSITION);

            case FAR_SIDE:
                return new StrategyParams(kShootPoseFarSide, kShootPoseFarSide, kShootParamFarSide,
                        kShootParamFarSide, k53to54Path, kShootPoseFarSideTo55Path,
                        Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90),
                        kShootPoseFarSideTo53Path, FieldConstants.NOTE_55_POSITION);
            default:
                throw new IllegalArgumentException("Invalid strategy for DallasAutoDrop53Routine");
        }
    }


    public static Command buildDrop53Command(DrivetrainSubsystem drivetrainSubsystem, Arm arm,
            Shooter shooter, Transfer transfer, Intake intake,
            Drop53Strategy strategy, Rotation2d fallbackRotation53) {
        StrategyParams params = getStrategyParams(strategy);
        return new SequentialCommandGroup(
                // Prepare routine
                AutoCommandFactory.buildPrepCommand(drivetrainSubsystem,
                        ShootingParameters.BELOW_SPEAKER, kStartPathDallas, arm, shooter, transfer),

                // Move to 53, intake and shoot 32 along the way
                buildIntakeShootWhileMovingCommandWithIntake53(drivetrainSubsystem, arm,
                        shooter, transfer, intake, GamePieceProcessor.getInstance(),
                        kStartPathDallas, kShootParam32, fallbackRotation53),

                Commands.either(
                        new TurnToHeadingCommand(drivetrainSubsystem, new Rotation2d()), 
                        new TurnToHeadingCommand(drivetrainSubsystem, Rotation2d.fromDegrees(180)), 
                        ()-> DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get()==Alliance.Blue),

                // Drop 53 and Get first note
                drop53ThenBuildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter,
                        transfer, intake, GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.firstNotePath)),
                        params.firstNoteRotation),

                // Score first note
                AutoBuilder
                        .pathfindToPoseFlipped(params.shootPoseFirstNote,
                                PathfindConstants.constraints)
                        .deadlineWith(
                                new SetArmAngleCommand(arm, params.shootParamsFirstNote.angle_deg),
                                new SetShooterTargetCommand(shooter,
                                        params.shootParamsFirstNote.speed_rps)),

                // Get second note
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter,
                        transfer, intake, GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.secondNotePath)),
                        null),

                // Score second note
                AutoBuilder
                        .pathfindToPoseFlipped(params.shootPoseSecondNote,
                                PathfindConstants.constraints)
                        .deadlineWith(
                                new SetArmAngleCommand(arm, params.shootParamsSecondNote.angle_deg),
                                new SetShooterTargetCommand(shooter,
                                        params.shootParamsSecondNote.speed_rps)),

                // Go to dropped 53
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter,
                        transfer, intake, GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(params.gotoDrop53Path)),
                        null),

                // Score 53
                AutoBuilder
                        .pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                        .deadlineWith(new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                                new SetShooterTargetCommand(shooter,
                                        kShootParamUnderStage.speed_rps)),

                // Move to midline
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter,
                        transfer, intake, GamePieceProcessor.getInstance(),
                        AutoBuilder.pathfindToPoseFlipped(params.endChasePose,
                                PathfindConstants.constraints),
                        Rotation2d.fromDegrees(0))

        );
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
                  boolean deadline = AutoCommandFactory.isFieldPositionReached(drivetrainSubsystem, AutoCommandFactory.kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget);
                }),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(() -> AutoCommandFactory.isFieldPositionReached(drivetrainSubsystem, AutoCommandFactory.kMidFieldFenceX))
            .until(() -> transfer.isOmronDetected())
            .alongWith(new SetShooterTargetCommand(shooter, 17)),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note"))
        );
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
                // if note, it must be prepared, feed first

                new WaitCommand(0.0).andThen(new FeedCommand(transfer, shooter)),
                
                Commands.runOnce(() -> {shooter.stop(); SmartDashboard.putString("Auto Status", "dropped 53");}, shooter),
                new ParallelCommandGroup(
                    new SetArmAngleCommand(arm, ArmConstants.ARM_OBSERVE_ANGLE),
                    new IntakeCommand(intake, transfer))))
            .until(
                () -> {
                  /*
                   * Chase conditions
                   * 1. Deadline is whether we are far enough the field to 54
                   * 2. Has target is whether we have a target to chase
                   * 3. If we have a note, 53 has not been shot yet, we need to wait
                   */
                  boolean deadline = AutoCommandFactory.isFieldPositionReached(drivetrainSubsystem, AutoCommandFactory.kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget && !transfer.isOmronDetected());
                }),
        // new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing
        // note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(
                () -> {
                  boolean midbar = AutoCommandFactory.isFieldPositionReached(drivetrainSubsystem, AutoCommandFactory.kMidFieldFenceX);
                  if (midbar)
                    SmartDashboard.putString("Auto Status", "chase interrupt because midfield bar");
                  return midbar;
                })
            // .until(() -> {
            // if (intake.isOmronDetected()) {
            // timer.start();
            // } else {
            // timer.stop();
            // timer.reset();
            // }
            // return timer.hasElapsed(0.5);
            // })
            .unless(() -> transfer.isOmronDetected()),
        // new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking
        // for note")),
        Commands.either(
            new WaitCommand(0),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotating to find note")),
                new TurnToHeadingCommand(drivetrainSubsystem, findNoteHeading).deadlineWith(new IntakeCommand(intake, transfer)).until(()->gamePieceProcessor.getClosestGamePieceInfo().isPresent()),
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotation Finished")),
                new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
                    .until(
                        () -> {
                          boolean midbar = AutoCommandFactory.isFieldPositionReached(drivetrainSubsystem, AutoCommandFactory.kMidFieldFenceX);
                          if (midbar)
                            SmartDashboard.putString(
                                "Auto Status", "chase interrupt because midfield bar");
                          return midbar;
                        })).until(()->transfer.isOmronDetected()), // end anytime if note is intaked
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
