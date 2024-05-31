package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Set a single velocity target for the shooter subsystem, finishes when the target is reached.
 * velocity target = 0 means stop shooter.
 */
public class SetShooterTargetCommand extends Command {
  private final Shooter mShooter;
  private final double shooterTargetRPS;

  public SetShooterTargetCommand(Shooter shooter, double targetRPS) {
    mShooter = shooter;
    this.shooterTargetRPS = targetRPS;

    addRequirements(mShooter);
  }

  @Override
  public void initialize() {
    mShooter.setTargetVelocity(shooterTargetRPS);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return mShooter.isReadyToShoot();
  }
}
