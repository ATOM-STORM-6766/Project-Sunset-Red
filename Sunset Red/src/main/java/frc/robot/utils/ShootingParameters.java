package frc.robot.utils;

public class ShootingParameters {

  public static final ShootingParameters BELOW_SPEAKER = new ShootingParameters(75, 59.0);
  // amp shooting angle and speed
  public static final ShootingParameters AMP_LOWSPEED = new ShootingParameters(11, 118);
  // amp prep state
  public static final ShootingParameters AMP_INTERMEDIATE_POS = new ShootingParameters(35, 90);
  // trap shooting state
  public static final ShootingParameters TRAP = new ShootingParameters(35, 64);

  public double speed_rps;
  public double angle_deg;

  /**
   * constructs a shooting paramter object, make sure speed_rps and angle_deg are legal
   *
   * @param speed_rps
   * @param angle_deg
   */
  public ShootingParameters(double speed_rps, double angle_deg) {
    this.speed_rps = Math.max(Math.min(speed_rps, 110), 0);
    this.angle_deg = Math.max(Math.min(angle_deg, 120.0), 26.0);
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) return true;
    if (obj == null || getClass() != obj.getClass()) return false;
    ShootingParameters other = (ShootingParameters) obj;
    return speed_rps == other.speed_rps && angle_deg == other.angle_deg;
  }
}
