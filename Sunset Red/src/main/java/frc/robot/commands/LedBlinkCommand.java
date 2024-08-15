package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

// this command make led blink for some time
// and finishes when given time passed
public class LedBlinkCommand extends Command {

    private final LED mLed;
    private final Color mBlinkColor;
    private final double mBlinkTime;

    private Timer stateTimer;

    public LedBlinkCommand(LED led, Color blinkColor, double blinkTime) {
        mLed = led;
        mBlinkColor = blinkColor;
        mBlinkTime = blinkTime;
        addRequirements(mLed);

        stateTimer = new Timer();
    }

    @Override
    public void initialize() {
        stateTimer.reset();
        stateTimer.start();
    }

    @Override
    public void execute() {
        mLed.setBlinkingColor(mBlinkColor);
    }

    @Override
    public void end(boolean interrupted) {
        stateTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return stateTimer.hasElapsed(mBlinkTime);
    }
    
    
}
