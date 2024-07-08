package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
import java.util.Optional;

public class SectionBasedAutoCommand extends SequentialCommandGroup {
  private Intake sIntake;
  private Shooter sShooter;
  private Arm sArm;
  private Transfer sTransfer;
  private DrivetrainSubsystem sDrivetrainSubsystem;

  private boolean reachedPointB = false;

  public SectionBasedAutoCommand(
      Intake intake,
      Shooter shooter,
      Arm arm,
      Transfer transfer,
      DrivetrainSubsystem drivetrainSubsystem) {
    this.sIntake = intake;
    this.sShooter = shooter;
    this.sArm = arm;
    this.sTransfer = transfer;
    this.sDrivetrainSubsystem = drivetrainSubsystem;

    addCommands(
        new SequentialCommandGroup(
            sDrivetrainSubsystem.runZeroingCommand(),
            new InstantCommand(
                () -> {
                  Optional<Alliance> a = DriverStation.getAlliance();
                  PathPlannerPath firstPath = PathPlannerPath.fromPathFile("HomeA to ObsA");
                  if (a.isPresent() && a.get() == Alliance.Red) {
                    firstPath = firstPath.flipPath();
                  }
                  sDrivetrainSubsystem.setPose(firstPath.getPreviewStartingHolonomicPose());
                })),
        new SequentialCommandGroup(
            // Initial setup and preload shot
            new ParallelCommandGroup(
                new SetArmAngleCommand(sArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                new SetShooterTargetCommand(sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
            new FeedCommand(sTransfer),
            new InstantCommand(() -> sShooter.stop()),
            new SetArmAngleCommand(sArm, Constants.ArmConstants.ARM_REST_ANGLE),

            // Move to first observe point and attempt to chase note
            buildPath("HomeA to ObsA").deadlineWith(new IntakeCommand(sIntake, sTransfer)),
            new ChaseNoteCommand(
                sDrivetrainSubsystem, GamePieceProcessor.getInstance(), sIntake, sTransfer),

            // If chase fails, move to second observe point
            new ConditionalCommand(
                new SequentialCommandGroup(
                    buildPath("ObsA to ObsB"),
                    new ChaseNoteCommand(
                        sDrivetrainSubsystem, GamePieceProcessor.getInstance(), sIntake, sTransfer),
                    new InstantCommand(() -> reachedPointB = true)),
                new InstantCommand(),
                () -> !sIntake.isOmronDetected()),

            // Return to shooting position
            new ConditionalCommand(
                    buildPath("ReturnB to HomeB"),
                    buildPath("ReturnA to HomeA"),
                    () -> reachedPointB // Choose path based on which route it took
                    )
                .alongWith(
                    new SetShooterTargetCommand(
                        sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps))
                .alongWith(
                    new SetArmAngleCommand(sArm, ShootingParameters.BELOW_SPEAKER.angle_deg)),

            // Shoot if note was picked up
            new ConditionalCommand(
                new FeedCommand(sTransfer),
                new InstantCommand(),
                () -> sTransfer.isOmronDetected()),
            new InstantCommand(() -> sShooter.stop())));
  }

  private Command buildPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }
}
