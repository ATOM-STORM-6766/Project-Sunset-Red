package frc.robot.auto.modes;

import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.commands.ChaseNoteCommand;
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

public class NearMid52then51AutoCommand extends SequentialCommandGroup {
    
  private static final String kStartPathName = "nearTo52";

  // dedicated shooting pose for this auto
  private static final Pose2d kShootPose = new Pose2d(4.2, 6.41, Rotation2d.fromDegrees(14.0));
  // dedicated shooting parameters (hard coded fornow) for this auto
  private static final ShootingParameters kShootParam = new ShootingParameters(75, 31);

  public NearMid52then51AutoCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Arm arm,
    Shooter shooter,
    Transfer transfer,
    Intake intake)
  {
    addCommands(
      AutoCommandFactory.buildPrepCommand(drivetrainSubsystem, 
        ShootingParameters.BELOW_SPEAKER, kStartPathName, arm, shooter, transfer),
      // chase 52
      AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(), 
        AutoBuilder.followPath(PathPlannerPath.fromPathFile(kStartPathName)), Rotation2d.fromDegrees(90.0)),
      // TODO : check 51 taken
      // shoot 52
      AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints)
        .deadlineWith(new SetArmAngleCommand(arm, kShootParam.angle_deg),
          new SetShooterTargetCommand(shooter, kShootParam.speed_rps)),
      // chase 51 no failure ver
      new ParallelDeadlineGroup(
              AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_51_POSITION, PathfindConstants.constraints),
                new SequentialCommandGroup(
                  // if note, it must be prepared, feed first
                  new SequentialCommandGroup(
                    new FeedCommand(transfer),
                    Commands.runOnce(() -> shooter.stop(), shooter)
                  ).onlyIf(() -> transfer.isOmronDetected()),
                  new ParallelCommandGroup(
                    new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                    new IntakeCommand(intake, transfer)
                  )
                ))
            .until(
                () -> {
                  boolean deadline = AutoCommandFactory.isChaseDeadlineReached(drivetrainSubsystem);
                  Optional<PhotonTrackedTarget> target =
                      GamePieceProcessor.getInstance().getClosestGamePieceInfo();
                  boolean hasTarget = target.isPresent();
                  SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                  SmartDashboard.putBoolean("Has Target", hasTarget);
                  return (deadline && hasTarget) || transfer.isOmronDetected();
                }),
      new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
      new ChaseNoteCommand(drivetrainSubsystem, intake, transfer, arm)
          .until(() -> AutoCommandFactory.isMidFieldFenceReached(drivetrainSubsystem))
            .unless(() -> transfer.isOmronDetected()),
      new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note")),
      // shoot 51 if we have it
      AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints)
        .deadlineWith(new SetArmAngleCommand(arm, kShootParam.angle_deg),
          new SetShooterTargetCommand(shooter, kShootParam.speed_rps))
        .onlyIf(() -> transfer.isOmronDetected()),
      // feed 52 go 54and55
      AutoCommandFactory.buildTake54Stop55Command(drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance())
    );
  }
}
