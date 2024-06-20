package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveHeadingController {
  private static final double DISABLE_TIME_LENGTH = 0.2; // Unit: seconds
  private static final double ALLOWABLE_ERROR = Units.degreesToRadians(1.5); // Unit: radians
  private Rotation2d targetHeading;
  private double targetVelocity;
  private double disabledTimestamp;

  private TrapezoidProfile.Constraints swerveRotateConstraints;
  private ProfiledPIDController swerveHeadingPID;

  public enum State {
    Off,
    On,
    TemporaryDisable
  }

  private State currentState = State.Off;

  public synchronized State getState() {
    return currentState;
  }

  private synchronized void setState(State newState) {
    currentState = newState;
  }

  public SwerveHeadingController() {
    swerveRotateConstraints =
        new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(300));
    swerveHeadingPID = new ProfiledPIDController(5.0, 0.0, 0.0, swerveRotateConstraints);
    targetHeading = new Rotation2d();
    swerveHeadingPID.setTolerance(ALLOWABLE_ERROR);
    swerveHeadingPID.enableContinuousInput(
        Units.degreesToRadians(-180), Units.degreesToRadians(180));
  }

  public synchronized void setTarget(Rotation2d target) {
    targetHeading = target;
    swerveHeadingPID.setGoal(targetHeading.getRadians());
    setState(State.On);
  }

  public synchronized void disable() {
    setState(State.Off);
  }

  public synchronized void temporarilyDisable() {
    setState(State.TemporaryDisable);
    disabledTimestamp = Timer.getFPGATimestamp();
  }

  public synchronized Rotation2d getTargetHeading() {
    return targetHeading;
  }

  public synchronized double getTargetHeadingVelocity() {
    return targetVelocity;
  }

  public synchronized double updateRotationCorrection(Rotation2d heading, double timestamp) {
    double correction = 0;
    switch (currentState) {
      default:
      case Off:
        break;
      case TemporaryDisable:
        targetHeading = heading;
        if (timestamp - disabledTimestamp >= DISABLE_TIME_LENGTH) {
          swerveHeadingPID.reset(heading.getRadians());
          setTarget(heading);
        }
        break;
      case On:
        correction = swerveHeadingPID.calculate(heading.getRadians());
        targetVelocity = swerveHeadingPID.getSetpoint().velocity;
        correction += targetVelocity;

        if (swerveHeadingPID.atGoal()) {
          correction = 0.0;
        }
        break;
    }

    SmartDashboard.putNumber(
        "Heading stabilization position setpoint", swerveHeadingPID.getSetpoint().position);
    // SmartDashboard.putNumber("Heading stabilization velocity setpoint",
    // swerveHeadingPID.getSetpoint().velocity);
    SmartDashboard.putNumber(
        "Heading stabilization position error", swerveHeadingPID.getPositionError());

    // SmartDashboard.putString("Heading State", currentState.toString());
    // SmartDashboard.putNumber("Heading Correction", correction);
    return correction;
  }

  public synchronized void reset() {
    reset(new Rotation2d());
  }

  public synchronized void reset(Rotation2d measurement) {
    swerveHeadingPID.reset(measurement.getRadians());
    setTarget(measurement);
  }
}
