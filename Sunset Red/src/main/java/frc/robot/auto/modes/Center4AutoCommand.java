package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;

public class Center4AutoCommand extends SequentialCommandGroup {

  private Intake mIntake;
  private Shooter mShooter;
  private Arm mArm;
  private Transfer mTransfer;

  public Center4AutoCommand(
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
                new InstantCommand( // maybe don't need, will use vision to override
                    () -> {
                      Optional<Alliance> a = DriverStation.getAlliance();
                      PathPlannerPath firstPath = PathPlannerPath.fromPathFile("homeTo51");
                      if (a.isPresent() && a.get() == Alliance.Red) {
                        firstPath = firstPath.flipPath();
                      }
                      drivetrainSubsystem.setPose(firstPath.getPreviewStartingHolonomicPose());
                    })),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetArmAngleCommand(mArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                    new SetShooterTargetCommand(
                        mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                new FeedCommand(mTransfer),
                new InstantCommand(
                    () -> {
                      mShooter.stop();
                    }))),
        // go 51 and back shoot
        buildPath("homeTo51").deadlineWith(new IntakeCommand(mIntake, mTransfer)),
        buildPath("51ToHome")
            .alongWith(
                new SetShooterTargetCommand(mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .andThen(new FeedCommand(mTransfer).onlyIf(() -> mTransfer.isOmronDetected()))
            .andThen(
                new InstantCommand(
                    () -> {
                      mShooter.stop();
                    })),
        // go 52 and back shoot
        buildPath("homeTo52").deadlineWith(new IntakeCommand(mIntake, mTransfer)),
        buildPath("52ToHome")
            .alongWith(
                new SetShooterTargetCommand(mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .andThen(new FeedCommand(mTransfer).onlyIf(() -> mTransfer.isOmronDetected()))
            .andThen(
                new InstantCommand(
                    () -> {
                      mShooter.stop();
                    })),
        // go 53 and back shoot
        buildPath("homeTo53").deadlineWith(new IntakeCommand(mIntake, mTransfer)),
        buildPath("53ToHome")
            .alongWith(
                new SetShooterTargetCommand(mShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
            .andThen(new FeedCommand(mTransfer).onlyIf(() -> mTransfer.isOmronDetected()))
            .andThen(
                new InstantCommand(
                    () -> {
                      mShooter.stop();
                      mArm.stop();
                    })));
  }

  private Command buildPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }
}
