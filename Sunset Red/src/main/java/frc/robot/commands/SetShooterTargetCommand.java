package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Shooter;
import java.util.Map;
/**
 * Set a single velocity target for the shooter subsystem, finishes when the target is reached.
 * velocity target = 0 means stop shooter.
 */
public class SetShooterTargetCommand extends Command {
  private final Shooter mShooter;
  private final double shooterTargetRPS;
  private boolean isFinished;
  private static GenericEntry speedEntry= Shuffleboard.getTab("SmartDashboard").add("speed",30).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",30,"max",80)).getEntry();

  private static final double STABLIZE_TIME = 0.1;
  private DualEdgeDelayedBoolean spinStablized =
      new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), STABLIZE_TIME, EdgeType.RISING);
  private static final double SHOOT_THRESHOLD_RPS = 8.0; // lower than this velocity, game piece
  // will get stuck
  private static final double ERR_TOL =
      5.0; // spin velocity error tolerance (rps), increased to 5 for new shooter

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
    mShooter.setTargetVelocity(speedEntry.getDouble(30));
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
            && Math.abs(mainMotorVelocity - speedEntry.getDouble(30)) < ERR_TOL
            && Math.abs(followerVelocity - speedEntry.getDouble(30)) < ERR_TOL)) {
      // ready to shoot
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
