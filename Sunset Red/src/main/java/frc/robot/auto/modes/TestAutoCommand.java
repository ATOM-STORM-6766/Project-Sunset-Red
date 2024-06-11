package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestAutoCommand extends SequentialCommandGroup {
  public TestAutoCommand(DrivetrainSubsystem mDrivetrainSubsystem) {
    PathPlannerPath startPath = PathPlannerPath.fromPathFile("Example Path");
    addCommands(
        mDrivetrainSubsystem.runZeroingCommand(),
        Commands.runOnce(
            () -> mDrivetrainSubsystem.setPose(startPath.getPreviewStartingHolonomicPose())),
        AutoBuilder.followPath(startPath));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
