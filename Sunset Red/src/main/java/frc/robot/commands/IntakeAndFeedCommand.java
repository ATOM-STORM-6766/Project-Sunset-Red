package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeAndFeedCommand extends Command {
  private final Intake sIntake;
  private final Transfer sTransfer;

  public IntakeAndFeedCommand(Intake intake, Transfer transfer) {
    sIntake = intake;
    sTransfer = transfer;
    addRequirements(sIntake, sTransfer);
  }

  @Override
  public void initialize() {
    sIntake.setIntake();
    sTransfer.setVoltage(Transfer.FEED_VOLTS);
  }

  @Override
  public void end(boolean interrupted) {
    sIntake.stop();
    sTransfer.stop();
  }

  @Override
  public boolean isFinished() {
    // never finish by itself
    return false;
  }
}
