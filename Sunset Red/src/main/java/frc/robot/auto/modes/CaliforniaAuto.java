package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class CaliforniaAuto extends SequentialCommandGroup {
  private static final String kStartPathName51 = "California nearTo51";
  private static final String kStartPathName52 = "California nearTo52";
  // shooting pose for this auto, add as many as needed
  // todo: these are hardcoded for now, need to be updated for vision tracking
  private static final Pose2d kShootPoseNearPodium =
      new Pose2d(4.5, 6.46, Rotation2d.fromDegrees(14.0));
  private static final Pose2d kShootPoseUnderStage =
      new Pose2d(4.5, 4.8, Rotation2d.fromDegrees(-14));

  // shooting parameters for this auto, we default at fastest speed
  // todo: these are hardcoded for now, need to be updated for vision tracking
  private static final ShootingParameters kShootParam = new ShootingParameters(75, 33);

  public CaliforniaAuto(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Transfer transfer,
      Intake intake,
      boolean startWith52) {
    String initialPath = startWith52 ? kStartPathName52 : kStartPathName51;
    Command secondPath =
        startWith52
            ? AutoBuilder.pathfindToPoseFlipped(
                FieldConstants.NOTE_51_POSITION, PathfindConstants.constraints)
            : AutoBuilder.pathfindToPoseFlipped(
                FieldConstants.NOTE_52_POSITION, PathfindConstants.constraints);
    Rotation2d initialRotation =
        startWith52 ? Rotation2d.fromDegrees(90.0) : Rotation2d.fromDegrees(-90.0);
    Rotation2d secondRotation =
        startWith52 ? Rotation2d.fromDegrees(-90.0) : Rotation2d.fromDegrees(90.0);
    addCommands(
        AutoCommandFactory.buildPrepCommand(
            drivetrainSubsystem,
            ShootingParameters.BELOW_SPEAKER,
            initialPath,
            arm,
            shooter,
            transfer),
        // chase 51
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(initialPath)),
            initialRotation),
        // go to shooting pose and shoot 51
        AutoBuilder.pathfindToPoseFlipped(kShootPoseNearPodium, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, kShootParam.angle_deg),
                new SetShooterTargetCommand(shooter, kShootParam.speed_rps)),
        // chase 52
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            secondPath,
            secondRotation),
        // shoot 52
        AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, kShootParam.angle_deg),
                new SetShooterTargetCommand(shooter, kShootParam.speed_rps)),

        // chase 53
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.pathfindToPoseFlipped(
                FieldConstants.NOTE_53_POSITION, PathfindConstants.constraints),
            Rotation2d.fromDegrees(-90.0)),

        // shoot 53
        AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
            .deadlineWith(
                new SetArmAngleCommand(arm, kShootParam.angle_deg),
                new SetShooterTargetCommand(shooter, kShootParam.speed_rps)),
        // go to centerline, for now we just go to 54
        AutoCommandFactory.buildPathThenChaseNoteCommand(
            drivetrainSubsystem,
            arm,
            shooter,
            transfer,
            intake,
            GamePieceProcessor.getInstance(),
            AutoBuilder.pathfindToPoseFlipped(
                FieldConstants.NOTE_54_POSITION, PathfindConstants.constraints),
            Rotation2d.fromDegrees(-90.0)),
        new FeedCommand(transfer, shooter));
  }
}
