package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Shooter;

/**
 * Set a single velocity target for the shooter subsystem, finishes when the target is reached.
 * velocity target = 0 means stop shooter.
 */
public class SetShooterTargetCommand extends Command {
  private final Shooter mShooter;
  private final double shooterTargetRPS;
  private boolean isFinished;

  private static final double STABLIZE_TIME = 0.1;
  private DualEdgeDelayedBoolean spinStablized =
      new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), STABLIZE_TIME, EdgeType.RISING);
  private static final double SHOOT_THRESHOLD_RPS =
      20.0; // lower than this velocity, game piece will get stuck
  private static final double ERR_TOL = 2.0; // spin velocity error tolerance (rps)

  /**
   * @brief sets the shooter to a target rps, finishes when target reached, do not interact with
   *     transfer subsystem
   */
  public SetShooterTargetCommand(Shooter shooter, double targetRPS) {
    mShooter = shooter;
    this.shooterTargetRPS = targetRPS;
    addRequirements(mShooter);
  }

  @Override
  public void initialize() {
    mShooter.setTargetVelocity(shooterTargetRPS);
    isFinished = false; // this has to be here otherwise it will not reset on every trigger
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public void execute() {
    double mainMotorVelocity = mShooter.getMainMotorVelocity();
    double followerVelocity = mShooter.getFollowerVelocity();
    if (spinStablized.update(
        Timer.getFPGATimestamp(),
        mShooter.getMainMotorVelocity() > SHOOT_THRESHOLD_RPS
            && mShooter.getFollowerVelocity() > SHOOT_THRESHOLD_RPS
            && Math.abs(mainMotorVelocity - shooterTargetRPS) < ERR_TOL
            && Math.abs(followerVelocity - shooterTargetRPS) < ERR_TOL)) {
      // ready to shoot
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
