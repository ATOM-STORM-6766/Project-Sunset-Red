package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class OuttakeCommand extends Command {
  private final Intake mIntake;
  private final Transfer mTransfer;
  private double start_time;
  private final double OUTTAKE_TIME = 1.5;

  public OuttakeCommand(Intake intake, Transfer transfer) {
    mIntake = intake;
    mTransfer = transfer;
    addRequirements(mIntake, mTransfer);
  }

  @Override
  public void initialize() {
    start_time = Timer.getFPGATimestamp();

    mIntake.setOuttake();
    mTransfer.setVoltage(Transfer.OUTTAKE_VOLTS);
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.stop();
    mTransfer.stop();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - start_time > OUTTAKE_TIME; // timeout finish
  }
}
