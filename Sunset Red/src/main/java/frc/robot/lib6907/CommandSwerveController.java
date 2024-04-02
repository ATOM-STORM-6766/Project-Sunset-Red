package frc.robot.lib6907;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandSwerveController extends CommandXboxController {
  /*
   * Field origin always stays with blue alliance
   * TRANS_DIR = -1.0 for blue and 1.0 for red
   */
  private double trans_dir = -1.0;

  public CommandSwerveController(int port) {
    super(port);
  }

  private static final double DEADBAND = 0.1; // percent
  private static final double ROTATE_DEADBAND = 0.85; // percent
  private static final double NEAR_POLE_DRIVE = 7; // degrees
  private static final double NEAR_POLE_TURN = 7; // degrees
  private final SlewRateLimiter mRampX =
      new SlewRateLimiter(5); // (1 / seconds_from_neutral_to_full)
  private final SlewRateLimiter mRampY =
      new SlewRateLimiter(5); // (1 / seconds_from_neutral_to_full)
  private final SlewRateLimiter mRampTurn =
      new SlewRateLimiter(5); // (1 / seconds_from_neutral_to_full)

  public Translation2d getDriveVector(boolean robot_oriented) {
    double xspeed, yspeed;
    if (robot_oriented) {
      xspeed = mRampX.calculate(-1.0 * getLeftY());
      yspeed = mRampY.calculate(-1.0 * getLeftX());
    } else { // field oriented
      xspeed = mRampX.calculate(trans_dir * getLeftY());
      yspeed = mRampY.calculate(trans_dir * getLeftX());
    }
    Translation2d dv = new Translation2d(xspeed, yspeed);
    if (dv.getNorm() < DEADBAND) {
      return new Translation2d();
    }
    // near pole adjust
    Rotation2d dvangle = dv.getAngle();
    Rotation2d nearpole = Util.nearestPole(dvangle);
    if (Math.abs(dvangle.getDegrees() - nearpole.getDegrees()) < NEAR_POLE_DRIVE) {
      dv = new Translation2d(dv.getNorm(), nearpole);
    }

    return dv;
  }

  public double getDriveTargetAngle() {
    Translation2d headingVector =
        new Translation2d(trans_dir * getRightY(), trans_dir * getRightX());
    if (headingVector.getNorm() < ROTATE_DEADBAND) {
      return Double.POSITIVE_INFINITY;
    }

    // near pole adjust
    Rotation2d headingAngle = headingVector.getAngle();
    Rotation2d nearpole = Util.nearestPole(headingAngle);
    if (Math.abs(headingAngle.getDegrees() - nearpole.getDegrees()) < NEAR_POLE_TURN) {
      headingAngle = nearpole;
    }

    return headingAngle.getDegrees();
  }

  public double getRawChangeRate() {
    double rate = Util.eliminateDeadband(getLeftTriggerAxis() - getRightTriggerAxis());
    return mRampTurn.calculate(rate);
  }

  public boolean getPigeonReset() {
    return getHID().getStartButton();
  }

  public boolean getSlowMode() {
    return getHID().getRightBumper();
  }

  // when driver home pressed
  public void setTransDir(boolean by_alliance) {
    if (by_alliance && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      trans_dir = 1;
    } else {
      trans_dir = -1;
    }
  }
}
