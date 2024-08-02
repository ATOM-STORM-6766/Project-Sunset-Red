package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.commands.ChaseNoteCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeAndFeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoCommandFactory {

  // the starting pose for near home. Heading set for shooting preload in-place.
  public static final Pose2d kNearStart = new Pose2d(1.27, 6.55, Rotation2d.fromDegrees(30.0));

  // upon robot pose x reach kChaseNoteDeadlineX we start chase note
  // for reference: wing line ~5.8m
  // TODO: THIS SHOULD BE 6.5-7.0
  private static final double kChaseNoteDeadlineX = 6.3;

  // upon robot pose x reach kMidFieldFenceX the robot has crossed the midfield
  // completely
  // which is likely to violate rules, so chase note will be cancelled.
  private static final double kMidFieldFenceX = 8.5;

  /**
   * This is the start command for any auto routine. It zeroes the drivetrain and
   * set initial pose
   * along with shooting the preload. This command has proper logging for command
   * scheduling.
   *
   * @param drivetrainSubsystem
   * @param shootingParameters  the preload shooting parameter
   * @param startPathName       the path file name that used as the start of the
   *                            path (to set initial
   *                            pose)
   * @param arm
   * @param shooter
   * @param transfer
   * @return Command
   */
  public static Command buildPrepCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      ShootingParameters shootingParameters,
      String startPathName,
      Arm arm,
      Shooter shooter,
      Transfer transfer) {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
            drivetrainSubsystem.runZeroingCommand(),
            new InstantCommand(
                () -> {
                  Optional<Alliance> currentAlliance = DriverStation.getAlliance();
                  PathPlannerPath firstPath = PathPlannerPath.fromPathFile(startPathName);
                  if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
                    firstPath = firstPath.flipPath();
                  }
                  drivetrainSubsystem.setPose(firstPath.getPreviewStartingHolonomicPose());
                  SmartDashboard.putString("Auto Status", "Finished prepare command");
                })),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArmAngleCommand(arm, shootingParameters.angle_deg),
                new SetShooterTargetCommand(shooter, shootingParameters.speed_rps)),
            new FeedCommand(transfer),
            new InstantCommand(
                () -> {
                  shooter.stop();
                })));
  }

  /**
   * This is the standard command for finding and chasing note in auto routine. It
   * follows the
   * pathCommand until note is seen, then chase note. If the first chase attempt
   * failed (no note in
   * transfer) we do a findNoteHeading turn and then chase again. This command
   * takes care of mid
   * field fencing. After this command you can do what you want with the note (if
   * you have it!)
   *
   * @param drivetrainSubsystem
   * @param arm
   * @param shooter
   * @param transfer
   * @param intake
   * @param gamePieceProcessor
   * @param pathCommand         the command for finding the path to a specific
   *                            note. Can be pathfollow or
   *                            pathfind or followthenfind, whatever you want.
   * @param findNoteHeading     the heading for finding note. For example with
   *                            near-side auto
   *                            (california) give me -90.0 degree and for far-side
   *                            auto(southern-cross) give 90.0 degrees.
   * @return
   */
  public static Command buildPathThenChaseNoteCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      GamePieceProcessor gamePieceProcessor,
      Command pathCommand,
      Rotation2d findNoteHeading) {
    Timer timer = new Timer();
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            pathCommand,
            new SequentialCommandGroup(
                // if note, it must be prepared, feed first
                new SequentialCommandGroup(
                    new FeedCommand(transfer), Commands.runOnce(() -> shooter.stop(), shooter)),
                new ParallelCommandGroup(
                    new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                    new IntakeCommand(intake, transfer))))
            .until(
                () -> {
                  /*
                   * Chase conditions
                   * 1. Deadline is whether we are far enough the field to 54
                   * 2. Has target is whether we have a target to chase
                   * 3. If we have a note, we can stop chasing
                   */
                  boolean deadline = isFieldPositionReached(drivetrainSubsystem, kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget); // || transfer.isOmronDetected();
                }),
        // new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing
        // note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(
                () -> {
                  boolean midbar = isFieldPositionReached(drivetrainSubsystem, kMidFieldFenceX);
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
                new TurnToHeadingCommand(drivetrainSubsystem, findNoteHeading),
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotation Finished")),
                new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
                    .until(
                        () -> {
                          boolean midbar = isFieldPositionReached(drivetrainSubsystem, kMidFieldFenceX);
                          if (midbar)
                            SmartDashboard.putString(
                                "Auto Status", "chase interrupt because midfield bar");
                          return midbar;
                        })),
            () -> {
              Boolean hasNote = transfer.isOmronDetected();
              SmartDashboard.putBoolean("Has Note", hasNote);
              if (hasNote) {
                SmartDashboard.putString("Auto Status", "hasnote=" + hasNote.toString());
              }
              return hasNote;
            }));
  }

  private static final Pose2d kStop55Pose = new Pose2d(7.72, 1.24, Rotation2d.fromDegrees(-45.0));

  public static Command buildTake54Stop55Command(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      GamePieceProcessor gamePieceProcessor) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("take54EndPath"), PathfindConstants.constraints),
            new SequentialCommandGroup(
                // if note, it must be prepared, feed first
                new SequentialCommandGroup(
                    new FeedCommand(transfer),
                    Commands.runOnce(() -> shooter.stop(), shooter))
                    .onlyIf(() -> transfer.isOmronDetected()),
                new ParallelCommandGroup(
                    new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                    new IntakeCommand(intake, transfer))))
            .until(
                () -> {
                  // deadline is whether we are far enough the field to 54
                  boolean deadline = (drivetrainSubsystem.getPose().getY() < 3.0);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget); // || transfer.isOmronDetected();
                }),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(() -> isFieldPositionReached(drivetrainSubsystem, kMidFieldFenceX))
            .unless(() -> transfer.isOmronDetected()),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note")),
        Commands.either(
            // stop 55 and ok
            AutoBuilder.pathfindToPose(kStop55Pose, PathfindConstants.constraints),
            // if no note, just take 55
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotating to find note")),
                new TurnToHeadingCommand(drivetrainSubsystem, Rotation2d.fromDegrees(-90.0)),
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotation Finished")),
                new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
                    .until(() -> isFieldPositionReached(drivetrainSubsystem, kMidFieldFenceX))),
            () -> {
              boolean hasNote = transfer.isOmronDetected();
              SmartDashboard.putBoolean("Has Note", hasNote);
              return hasNote;
            }));
  }

  public static boolean isFieldPositionReached(
      DrivetrainSubsystem drivetrainSubsystem, double threshold) {
    Optional<Alliance> a = DriverStation.getAlliance();
    double robotX = drivetrainSubsystem.getPose().getX();
    if (a.isPresent() && a.get() == Alliance.Red) { // red
      return robotX < 16.54 - threshold;
    } else { // blue
      return robotX > threshold;
    }
  }

  /**
   * This command is almost only for the OP-Robotics mid start auto. i.e. Dallas
   * Auto we follow path
   * and shoot 32 on the fly
   *
   * @param drivetrainSubsystem
   * @param arm
   * @param shooter
   * @param transfer
   * @param intake
   * @param gamePieceProcessor
   * @param pathName
   * @param shootParams
   * @param findNoteHeading
   * @return
   */
  public static Command buildIntakeShootWhileMovingCommand(
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
                    new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                    new IntakeCommand(intake, transfer))))
            .until(
                () -> {
                  boolean deadline = isFieldPositionReached(drivetrainSubsystem, kChaseNoteDeadlineX);
                  Optional<PhotonTrackedTarget> target = gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget);
                }),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
        new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(() -> isFieldPositionReached(drivetrainSubsystem, kMidFieldFenceX))
            .unless(() -> transfer.isOmronDetected()),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note")),
        Commands.either(
            new WaitCommand(0),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotating to find note")),
                new TurnToHeadingCommand(drivetrainSubsystem, findNoteHeading),
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotation Finished")),
                new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
                    .until(() -> isFieldPositionReached(drivetrainSubsystem, kMidFieldFenceX))),
            () -> {
              boolean hasNote = transfer.isOmronDetected();
              SmartDashboard.putBoolean("Has Note", hasNote);
              return hasNote;
            }));
  }

}
