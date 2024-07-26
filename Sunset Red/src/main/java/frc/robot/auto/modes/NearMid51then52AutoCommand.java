package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class NearMid51then52AutoCommand extends SequentialCommandGroup {

  private static final String kStartPathName = "nearTo51";

  // dedicated shooting pose for this auto
  private static final Pose2d kShootPose = new Pose2d(4.2, 6.41, Rotation2d.fromDegrees(14.0));
  // dedicated shooting parameters (hard coded fornow) for this auto
  private static final ShootingParameters kShootParam = new ShootingParameters(75, 31);

  public NearMid51then52AutoCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Arm arm,
    Shooter shooter,
    Transfer transfer,
    Intake intake)
  {

    addCommands(
      AutoCommandFactory.buildPrepCommand(drivetrainSubsystem, 
        ShootingParameters.BELOW_SPEAKER, kStartPathName, arm, shooter, transfer),
      // chase 51
      AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(), 
        AutoBuilder.followPath(PathPlannerPath.fromPathFile(kStartPathName)), Rotation2d.fromDegrees(-90.0)),
      // TODO : check 52 taken
      // shoot 51
      AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints)
        .deadlineWith(new SetArmAngleCommand(arm, kShootParam.angle_deg),
          new SetShooterTargetCommand(shooter, kShootParam.speed_rps)),
      // chase 52
      AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(), 
        AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_52_POSITION, PathfindConstants.constraints), Rotation2d.fromDegrees(-90.0)),
      // TODO : check 53 taken
      // shoot 52
      AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints)
        .deadlineWith(new SetArmAngleCommand(arm, kShootParam.angle_deg),
          new SetShooterTargetCommand(shooter, kShootParam.speed_rps)),
      // feed 52 go 54and55
      AutoCommandFactory.buildTake54Stop55Command(drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance())
    );
  }
}
