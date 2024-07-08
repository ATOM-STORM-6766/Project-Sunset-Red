package frc.robot.commands.AutoCommandGroups;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;

public class ZeroAndShootPreloadCommand extends ParallelCommandGroup {

  private Pose2d startPose;
  private ShootingParameters preloadShootingParameters;

  public ZeroAndShootPreloadCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Transfer transfer,
      Shooter shooter,
      Pose2d startPose2d,
      ShootingParameters preloadShootingParameters) {
    startPose = startPose2d;
    this.preloadShootingParameters = preloadShootingParameters;
    addCommands(
        new SequentialCommandGroup(
            drivetrainSubsystem.runZeroingCommand(),
            new InstantCommand( // maybe don't need, will use vision to override
                () -> {
                  Optional<Alliance> a = DriverStation.getAlliance();
                  if (a.isPresent() && a.get() == Alliance.Red) {
                    startPose = GeometryUtil.flipFieldPose(startPose);
                  }
                  drivetrainSubsystem.setPose(startPose);
                })),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArmAngleCommand(arm, this.preloadShootingParameters.angle_deg),
                new SetShooterTargetCommand(shooter, this.preloadShootingParameters.speed_rps)),
            new FeedCommand(transfer),
            new InstantCommand(
                () -> {
                  shooter.stop();
                }),
            new SetArmAngleCommand(arm, Constants.ArmConstants.ARM_REST_ANGLE)));
  }
}
