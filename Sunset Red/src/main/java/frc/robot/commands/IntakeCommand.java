package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeCommand extends Command {
  private final Intake mIntake;
  private final Transfer mTransfer;

  public IntakeCommand(Intake intake, Transfer transfer) {
    mIntake = intake;
    mTransfer = transfer;
    addRequirements(mIntake, mTransfer);
  }

  @Override
  public void initialize() {
    mIntake.setIntake();
    mTransfer.setVoltage(Transfer.INTAKE_VOLTS);
  }



  @Override
  public void execute() {
    if (mTransfer.isOmronDetected()) {
      mTransfer.stop();
      mIntake.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.stop();
    mTransfer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
