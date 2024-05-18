package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  private final Intake mIntake;

  public IntakeCommand(Intake intake) {
    mIntake = intake;
    addRequirements(mIntake);
  }

  @Override
  public void initialize() {
    mIntake.setIntake();
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
