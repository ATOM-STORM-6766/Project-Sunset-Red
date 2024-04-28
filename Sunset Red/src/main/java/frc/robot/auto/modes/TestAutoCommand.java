package frc.robot.auto.modes;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DrivetrainSubsystem;
public class TestAutoCommand extends SequentialCommandGroup{
    public TestAutoCommand(DrivetrainSubsystem mDrivetrainSubsystem){
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Auto");        
        addCommands(
            mDrivetrainSubsystem.runZeroingCommand(),
            Commands.runOnce(() -> mDrivetrainSubsystem.setPose(PathPlannerAuto.getStaringPoseFromAutoFile("Example Auto"))),
            AutoBuilder.followPath(pathGroup.get(0))

        );
    }
}
