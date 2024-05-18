package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;

public class FeedCommand extends Command {
    private final Transfer mTransfer;

    public FeedCommand(Transfer transfer) {
        mTransfer = transfer;
        addRequirements(mTransfer);
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
        return false;
    }
}
