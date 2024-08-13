package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class FeedCommand extends Command {
  private final Transfer mTransfer;

  private final double FEED_TIME = 0.5;
  private double start_time = Double.NaN;

  public FeedCommand(Transfer transfer, Shooter shooter) {
    mTransfer = transfer;
    addRequirements(mTransfer);
    addRequirements(
        shooter); // we add this because we don't want default command to slow down shooter while
                  // feeding
    // but this require further testing because sequential command will add all command
    // requirements.
  }

  @Override
  public void initialize() {
    start_time = Timer.getFPGATimestamp();
    mTransfer.setVoltage(Transfer.FEED_VOLTS);
  }

  @Override
  public void end(boolean interrupted) {
    mTransfer.stop();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - start_time > FEED_TIME;
  }
}
