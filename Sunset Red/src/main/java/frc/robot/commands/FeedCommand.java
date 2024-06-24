package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;

public class FeedCommand extends Command {
  private final Transfer mTransfer;

  private final double FEED_TIME = 1.0;
  private double start_time = Double.NaN;

  public FeedCommand(Transfer transfer) {
    mTransfer = transfer;
    addRequirements(mTransfer);
    start_time = Timer.getFPGATimestamp();
  }

  @Override
  public void initialize() {
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
