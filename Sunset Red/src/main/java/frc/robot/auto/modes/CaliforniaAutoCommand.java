package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.commands.VisionShootCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class CaliforniaAutoCommand extends SequentialCommandGroup {

  // private static final Pose2d kNearStart = new Pose2d(1.28, 6.41,
  // Rotation2d.fromDegrees(30.0));
  private static final Pose2d kShootPose = new Pose2d(4.2, 6.41, Rotation2d.fromDegrees(14.0));

  private static final Translation2d kZeroTranslation = new Translation2d();

  private DrivetrainSubsystem sDrivetrainSubsystem;
  private GamePieceProcessor sGamePieceProcessor;

  // near 30.0 degree start, preload and go 51 (between 31 32)
  // shoot 51 and go 52
  public CaliforniaAutoCommand(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
      Transfer transfer, Intake intake) {

    sDrivetrainSubsystem = drivetrainSubsystem;
    sGamePieceProcessor = GamePieceProcessor.getInstance();

    Command prepare = AutoCommandFactory.buildPrepCommand(drivetrainSubsystem,
        ShootingParameters.BELOW_SPEAKER, "California nearTo51", arm, shooter, transfer);

    Command goto51 = new SequentialCommandGroup(
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Starting goto51")),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("California nearTo51")),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Finished goto51")));

    Command pathFindto52 = new SequentialCommandGroup(
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Starting pathFindto52")),
        AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_52_POSITION,
            PathfindConstants.constraints, 0, 0),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Finished pathFindto52")));

    // build auto
    addCommands(
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Starting auto routine")),
        prepare, new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Going to 51")),
        AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter,
            transfer, intake, sGamePieceProcessor, goto51, Rotation2d.fromDegrees(-90.0)),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Finding path to shoot")),
        buildFindPathThenShoot(),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Shooting")),
        new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake,
            () -> kZeroTranslation).until(() -> !transfer.isOmronDetected())
                .andThen(new InstantCommand(() -> {
                  shooter.stop();
                  SmartDashboard.putString("Auto Status", "Finished shooting");
                })),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Going to 53")),
        AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter,
            transfer, intake, sGamePieceProcessor, pathFindto52, Rotation2d.fromDegrees(-90.0)),
        new InstantCommand(
            () -> SmartDashboard.putString("Auto Status", "Finding path to shoot (53)")),
        buildFindPathThenShoot(),
        new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Shooting (53)")),
        new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake,
            () -> kZeroTranslation).andThen(new InstantCommand(() -> {
              shooter.stop();
              SmartDashboard.putString("Auto Status", "Finished auto routine");
            })));
  }

  Command buildFindPathThenShoot() {
    return AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
