package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import java.util.function.Supplier;

// for led default, indicate if there is a note or not
public class LedDefaultCommand extends Command {

  private Color kHasNoteColor = new Color(0, 256, 0);

  private final LED mLed;
  private Supplier<Boolean> mTransferOmron;

  public LedDefaultCommand(LED led, Supplier<Boolean> transferOmron) {
    mLed = led;
    mTransferOmron = transferOmron;
    addRequirements(mLed);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (mTransferOmron.get()) {
      mLed.setSolidColor(kHasNoteColor);
    } else if (DriverStation.isDisabled()) {
      mLed.setSolidColor(Color.kRed);
    } else { // no note
      mLed.setMovingRedWhiteBlue();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
