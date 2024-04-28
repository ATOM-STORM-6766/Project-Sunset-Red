package frc.robot.auto.modes;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAutoCommand extends SequentialCommandGroup{
    public TestAutoCommand(){
        addCommands(
            
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"))
        );
    }
}
