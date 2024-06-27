package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class Home4AutoCommand extends SequentialCommandGroup {

  private Intake mIntake;
  private Shooter mShooter;
  private Arm mArm;
  private Transfer mTransfer;

  public Home4AutoCommand(
      Intake intake,
      Shooter shooter,
      Arm arm,
      Transfer transfer,
      DrivetrainSubsystem drivetrainSubsystem) {
    mIntake = intake;
    mShooter = shooter;
    mArm = arm;
    mTransfer = transfer;

    addCommands(
        // zeroing and shoot preload
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                drivetrainSubsystem.runZeroingCommand(),
                new InstantCommand(
                    () ->
                        drivetrainSubsystem.setPose(
                            PathPlannerPath.fromPathFile("homeTo31").flipPath().getPreviewStartingHolonomicPose()))),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetArmAngleCommand(mArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                    new SetShooterTargetCommand(
                        mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                new FeedCommand(mTransfer),
                new InstantCommand(() -> mShooter.stop()))),
        // go 31 and back shoot
        buildPath("homeTo31", true).deadlineWith(new IntakeCommand(mIntake, mTransfer)),
        buildPath("31ToHome", true)
            .alongWith(
                new SetShooterTargetCommand(mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .andThen(new FeedCommand(mTransfer).onlyIf(() -> mTransfer.isOmronDetected()))
            .andThen(new InstantCommand(() -> mShooter.stop())),
        // go 32 and back shoot
        buildPath("homeTo32", true).deadlineWith(new IntakeCommand(mIntake, mTransfer)),
        buildPath("32ToHome", true)
            .alongWith(
                new SetShooterTargetCommand(mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .andThen(new FeedCommand(mTransfer).onlyIf(() -> mTransfer.isOmronDetected()))
            .andThen(new InstantCommand(() -> mShooter.stop())),
        // go 32 and back shoot
        buildPath("homeTo33", true).deadlineWith(new IntakeCommand(mIntake, mTransfer)),
        buildPath("33ToHome", true)
            .alongWith(
                new SetShooterTargetCommand(mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .andThen(new FeedCommand(mTransfer).onlyIf(() -> mTransfer.isOmronDetected()))
            .andThen(new InstantCommand(() -> mShooter.stop())));
  }

  private Command buildPath(String pathName, boolean red_alliance) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    if (red_alliance){
        path = path.flipPath();
    }
    return AutoBuilder.followPath(path);
  }
}
