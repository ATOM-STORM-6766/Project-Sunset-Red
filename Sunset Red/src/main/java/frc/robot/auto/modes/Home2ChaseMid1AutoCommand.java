package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ChaseNoteStateMachineCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;
import java.util.List;
import java.util.Optional;

public class Home2ChaseMid1AutoCommand extends SequentialCommandGroup {

  private Intake sIntake;
  private Shooter sShooter;
  private Arm sArm;
  private Transfer sTransfer;
  private DrivetrainSubsystem sDrivetrainSubsystem;

  private static final Pose2d kNearHome = new Pose2d(0.68, 6.68, Rotation2d.fromDegrees(59.0));
  private static final Pose2d kWingWaypoint = new Pose2d(5.85, 7.10, Rotation2d.fromDegrees(0.0));
  private static final Pose2d kBetween5152Point =
      new Pose2d(8.26, 6.69, Rotation2d.fromDegrees(0.0));
  private static final Pose2d kCenterHome = new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(0.0));

  public Home2ChaseMid1AutoCommand(
      Intake intake,
      Shooter shooter,
      Arm arm,
      Transfer transfer,
      DrivetrainSubsystem drivetrainSubsystem) {
    sIntake = intake;
    sShooter = shooter;
    sArm = arm;
    sTransfer = transfer;
    sDrivetrainSubsystem = drivetrainSubsystem;

    addCommands(
        // zeroing and shoot preload
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                drivetrainSubsystem.runZeroingCommand(),
                new InstantCommand( // maybe don't need, will use vision to override
                    () -> {
                      Optional<Alliance> a = DriverStation.getAlliance();
                      Pose2d startPose = kNearHome;
                      if (a.isPresent() && a.get() == Alliance.Red) {
                        startPose = GeometryUtil.flipFieldPose(startPose);
                      }
                      drivetrainSubsystem.setPose(startPose);
                    })),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetArmAngleCommand(sArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                    new SetShooterTargetCommand(
                        sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                new FeedCommand(sTransfer),
                new InstantCommand(
                    () -> {
                      sShooter.stop();
                    }),
                new SetArmAngleCommand(arm, Constants.ArmConstants.ARM_REST_ANGLE))),
        // go to wing and then chase
        buildPathFromPoint(
                new GoalEndState(1.5, Rotation2d.fromDegrees(0)), kNearHome, kWingWaypoint)
            .deadlineWith(new IntakeCommand(intake, transfer)),
        new ChaseNoteStateMachineCommand(
            sDrivetrainSubsystem, GamePieceProcessor.getInstance(), sIntake, sTransfer),

        // go from between 5152 to waypoint
        buildPathFromPoint(
            new GoalEndState(1.5, kWingWaypoint.getRotation(), true),
            new Pose2d(kBetween5152Point.getTranslation(), Rotation2d.fromDegrees(-180.0)),
            new Pose2d(kWingWaypoint.getTranslation(), Rotation2d.fromDegrees(-180.0))),
        // go back and shoot
        buildPathFromPoint(
                new GoalEndState(0, kCenterHome.getRotation(), true),
                new Pose2d(kWingWaypoint.getTranslation(), Rotation2d.fromDegrees(-180.0)),
                new Pose2d(kCenterHome.getTranslation(), Rotation2d.fromDegrees(-180.0)))
            .alongWith(
                new SetShooterTargetCommand(sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .alongWith(new SetArmAngleCommand(arm, ShootingParameters.BELOW_SPEAKER.angle_deg))
            .andThen(new FeedCommand(sTransfer).onlyIf(() -> sTransfer.isOmronDetected()))
            .andThen(
                new InstantCommand(
                    () -> {
                      sShooter.stop();
                    })));
  }

  public Command buildPathFromPoint(GoalEndState goalEndState, Pose2d... poses) {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

    return AutoBuilder.followPath(
        new PathPlannerPath(
            bezierPoints, new PathConstraints(3.75, 4.0, 3 * Math.PI, 4 * Math.PI), goalEndState));
  }
}
