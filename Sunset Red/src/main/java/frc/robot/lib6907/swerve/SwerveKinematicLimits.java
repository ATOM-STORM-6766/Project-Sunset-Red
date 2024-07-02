package frc.robot.lib6907.swerve;

public class SwerveKinematicLimits {
  public double kMaxDriveVelocity; // m/s
  public double kMaxDriveAcceleration; // m/s^2
  public double kMaxSteeringVelocity; // rad/s

  public SwerveKinematicLimits(
      double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {
    this.kMaxDriveVelocity = maxDriveVelocity;
    this.kMaxDriveAcceleration = maxDriveAcceleration;
    this.kMaxSteeringVelocity = maxSteeringVelocity;
  }
}
