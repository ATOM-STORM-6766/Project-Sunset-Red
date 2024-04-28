package frc.robot.auto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.modes.*;


public class AutoModeSelector {
    private SendableChooser<Command> mChooser;

    private static AutoModeSelector sInstance;
    public static AutoModeSelector getInstance() {
        if (sInstance == null) sInstance = new AutoModeSelector();
        return sInstance;
    }
    private AutoModeSelector() {

    }

    public void pushChooser() {
        // init points
        mChooser = new SendableChooser<>();

        // tested
        mChooser.setDefaultOption("example", new TestAutoCommand());

        SmartDashboard.putData("AUTO CHOICES", mChooser);
    }

    public Command getSelected() {
        return mChooser.getSelected();
    }
    
}
