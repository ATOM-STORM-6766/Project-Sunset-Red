package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.commands.ChaseNoteStateMachineCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoCommandFactory {

  /**
   * This is the start command for any auto routine. It zeroes the drivetrain and set initial pose
   * along with shooting the preload. This command has proper logging for command scheduling.
   *
   * @param drivetrainSubsystem
   * @param shootingParameters the preload shooting parameter
   * @param startPathName the path file name that used as the start of the path (to set initial
   *     pose)
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
   * This is the standard command for finding and chasing note in auto routine. It follows the
   * pathCommand until note is seen, then chase note. If the first chase attempt failed (no note in
   * transfer) we do a findNoteHeading turn and then chase again. This command takes care of mid
   * field fencing. After this command you can do what you want with the note (if you have it!)
   *
   * @param drivetrainSubsystem
   * @param arm
   * @param shooter
   * @param transfer
   * @param intake
   * @param gamePieceProcessor
   * @param pathCommand the command for finding the path to a specific note. Can be pathfollow or
   *     pathfind or followthenfind, whatever you want.
   * @param findNoteHeading the heading for finding note. For example with near-side auto
   *     (california) give me -90.0 degree and for far-side auto(southern-cross) give 90.0 degrees.
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
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                pathCommand,
                new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                new IntakeCommand(intake, transfer))
            .until(
                () -> {
                  boolean deadline = isChaseDeadlineReached(drivetrainSubsystem);
                  Optional<PhotonTrackedTarget> target =
                      gamePieceProcessor.getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return deadline && hasTarget;
                }),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
        new ChaseNoteStateMachineCommand(drivetrainSubsystem, intake, transfer, arm)
            .until(() -> isMidFieldFenceReached(drivetrainSubsystem)),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note")),
        Commands.either(
            new WaitCommand(0),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotating to find note")),
                new TurnToHeadingCommand(drivetrainSubsystem, findNoteHeading),
                new InstantCommand(
                    () -> SmartDashboard.putString("Auto Status", "Rotation Finished")),
                new ChaseNoteStateMachineCommand(drivetrainSubsystem, intake, transfer, arm)
                    .until(() -> isMidFieldFenceReached(drivetrainSubsystem))),
            () -> {
              boolean hasNote = transfer.isOmronDetected();
              SmartDashboard.putBoolean("Has Note", hasNote);
              return hasNote;
            }));
  }

  // upon robot pose x reach kChaseNoteDeadlineX we start chase note
  // for reference: wing line ~5.8m
  // TODO: THIS SHOULD BE 6.5-7.0
  private static final double kChaseNoteDeadlineX = 4.8;

  private static boolean isChaseDeadlineReached(DrivetrainSubsystem drivetrainSubsystem) {
    Optional<Alliance> a = DriverStation.getAlliance();
    double robotX = drivetrainSubsystem.getPose().getX();
    if (a.isPresent() && a.get() == Alliance.Red) { // red
      return robotX < 16.54 - kChaseNoteDeadlineX;
    } else { // blue
      return robotX > kChaseNoteDeadlineX;
    }
  }

  // upon robot pose x reach kMidFieldFenceX the robot has crossed the midfield completely
  // which is likely to violate rules, so chase note will be cancelled.
  private static final double kMidFieldFenceX = 8.5;

  private static boolean isMidFieldFenceReached(DrivetrainSubsystem drivetrainSubsystem) {
    Optional<Alliance> a = DriverStation.getAlliance();
    double robotX = drivetrainSubsystem.getPose().getX();
    if (a.isPresent() && a.get() == Alliance.Red) { // red
      return robotX < 16.54 - kMidFieldFenceX;
    } else { // blue
      return robotX > kMidFieldFenceX;
    }
  }
}
