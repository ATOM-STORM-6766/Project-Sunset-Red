package frc.robot.lib6907;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;

/**
 * A Class that provides methods for reading input from an Xbox controller for controlling a swerve
 * drive.
 */
public class CommandSwerveController extends CommandXboxController {
  private static final double JOYSTICK_DEADBAND = 0.05;
  private static final double TRANSLATION_DEADBAND = 0.1; // percent
  private static final double ROTATION_DEADBAND = 0.85; // percent
  private static final double NEAR_POLE_DRIVE_DEGREES = 7;
  private static final double NEAR_POLE_TURN_DEGREES = 7;

  private final SlewRateLimiter translationXRateLimiter = new SlewRateLimiter(5); // (1 /
  // seconds_from_neutral_to_full)
  private final SlewRateLimiter translationYRateLimiter = new SlewRateLimiter(5); // (1 /
  // seconds_from_neutral_to_full)
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(5); // (1 /
  // seconds_from_neutral_to_full)

  private double translationDirectionMultiplier = 1.0; // 1.0 for blue, -1.0 for red

  /**
   * Constructs a new CommandSwerveController with the given port.
   *
   * @param port The port on the driver station that the controller is connected to.
   */
  public CommandSwerveController(int port) {
    super(port);
  }

  /**
   * Returns the translation vector for the swerve drive.
   *
   * @param driveMode the drive mode to use (robot-oriented or field-oriented)
   * @return the translation vector for the swerve drive
   */
  public Translation2d getDriveTranslation(DriveMode driveMode) {
    double xSpeed =
        driveMode == DriveMode.ROBOT_ORIENTED
            ? -getLeftY()
            : translationDirectionMultiplier * getLeftY();
    double ySpeed =
        driveMode == DriveMode.ROBOT_ORIENTED
            ? -getLeftX()
            : translationDirectionMultiplier * getLeftX();

    // this prevents the target velocity increase to fast, but should be included in setpoint
    // generator already
    double speedMultiplier = slowMode() ? 0.4 : 1.0;
    xSpeed = translationXRateLimiter.calculate(xSpeed * speedMultiplier);
    ySpeed = translationYRateLimiter.calculate(ySpeed * speedMultiplier);

    Translation2d translation = new Translation2d(xSpeed, ySpeed);

    if (translation.getNorm() < TRANSLATION_DEADBAND) {
      return new Translation2d();
    }

    Rotation2d translationAngle = translation.getAngle();
    Rotation2d nearestPole = nearestPole(translationAngle);

    if (Math.abs(translationAngle.minus(nearestPole).getDegrees()) < NEAR_POLE_DRIVE_DEGREES) {
      translation = new Translation2d(translation.getNorm(), nearestPole);
    }

    return translation;
  }

  /**
   * Returns an Optional containing the target angle for rotating the swerve drive, if available.
   *
   * @return An Optional containing the target angle in degrees, or an empty Optional if the
   *     rotation is below the deadband.
   */
  public Optional<Rotation2d> getDriveRotationAngle() {
    double rightY = translationDirectionMultiplier * getRightY();
    double rightX = translationDirectionMultiplier * getRightX();

    Translation2d rotationVector = new Translation2d(rightY, rightX);

    if (rotationVector.getNorm() < ROTATION_DEADBAND) {
      return Optional.empty();
    }

    Rotation2d rotationAngle = rotationVector.getAngle();
    Rotation2d nearestPole = nearestPole(rotationAngle);

    if (Math.abs(rotationAngle.minus(nearestPole).getDegrees()) < NEAR_POLE_TURN_DEGREES) {
      rotationAngle = nearestPole;
    }
    return Optional.of(rotationAngle);
  }

  /**
   * Returns the raw rotation rate from the controller triggers.
   *
   * @return The rotation rate.
   */
  public double getRawRotationRate() {
    double rate = applyDeadband(getLeftTriggerAxis() - getRightTriggerAxis());
    return rotationRateLimiter.calculate(rate);
  }

  /**
   * Returns whether the Pigeon IMU should be reset.
   *
   * @return True if the Pigeon IMU should be reset, false otherwise.
   */
  public boolean shouldResetPigeon() {
    return getHID().getStartButton();
  }

  /**
   * Returns whether the swerve drive should be in slow mode.
   *
   * @return True if the swerve drive should be in slow mode, false otherwise.
   */
  public boolean slowMode() {
    return getHID().getRightBumper();
  }

  /**
   * Sets the translation direction multiplier based on the alliance color.
   *
   * @param useAllianceColor Whether to set the direction based on the alliance color.
   */
  public void setTranslationDirection(boolean useAllianceColor) {
    if (useAllianceColor && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      translationDirectionMultiplier = 1.0;
    } else {
      translationDirectionMultiplier = -1.0;
    }
  }

  /**
   * Returns the nearest pole angle to the given rotation.
   *
   * <p>The nearest pole is determined by comparing the absolute values of the cosine and sine of
   * the rotation angle. If the absolute cosine is greater, the nearest pole is along the x-axis (0
   * or 180 degrees). If the absolute sine is greater, the nearest pole is along the y-axis (90 or
   * 270 degrees).
   *
   * @param rotation The rotation to find the nearest pole for.
   * @return The nearest pole angle.
   */
  private static Rotation2d nearestPole(Rotation2d rotation) {
    double poleSin =
        Math.abs(rotation.getCos()) > Math.abs(rotation.getSin())
            ? 0.0
            : Math.signum(rotation.getSin());
    double poleCos =
        Math.abs(rotation.getCos()) > Math.abs(rotation.getSin())
            ? Math.signum(rotation.getCos())
            : 0.0;
    return new Rotation2d(poleCos, poleSin);
  }

  /**
   * Applies a deadband to the given input value.
   *
   * @param input The input value.
   * @return The input value with the deadband applied, or 0.0 if the input is within the deadband.
   */
  private static double applyDeadband(double input) {
    return Math.abs(input) < JOYSTICK_DEADBAND ? 0.0 : input;
  }

  /** Represents the drive mode of the swerve drive. */
  public enum DriveMode {
    ROBOT_ORIENTED,
    FIELD_ORIENTED
  }

  /**
   * Returns whether the swerve drive should be in robot-relative mode.
   *
   * @return True if the swerve drive should be in robot-relative mode, false otherwise.
   */
  public DriveMode isRobotRelative() {
    return getHID().getLeftBumper() ? DriveMode.ROBOT_ORIENTED : DriveMode.FIELD_ORIENTED;
  }
}
