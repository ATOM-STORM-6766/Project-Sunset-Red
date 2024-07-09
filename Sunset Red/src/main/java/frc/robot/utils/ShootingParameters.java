package frc.robot.utils;

public class ShootingParameters {

  public static final ShootingParameters BELOW_SPEAKER = new ShootingParameters(85, 59.0);
  public static final ShootingParameters AMP = new ShootingParameters(25, 115);
  public static final ShootingParameters AGAINST_ENEMY = new ShootingParameters(85, 48.0);
  public static final ShootingParameters PODIUM_SHOOT = new ShootingParameters(85, 44.5);
  public static final ShootingParameters FAR_SHOOT = new ShootingParameters(85, 26.2);
  public static final ShootingParameters NEAR_SHOOT = new ShootingParameters(65, 26.2);

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
