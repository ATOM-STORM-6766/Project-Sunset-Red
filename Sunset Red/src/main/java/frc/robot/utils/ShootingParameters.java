package frc.robot.utils;

public class ShootingParameters {

  public static final ShootingParameters BELOW_SPEAKER = new ShootingParameters(75, 59);
  public static final ShootingParameters AMP_HIGHSPEED = new ShootingParameters(35, 118);
  public static final ShootingParameters AMP_LOWSPEED = new ShootingParameters(11, 118);
  public static final ShootingParameters AMP_INTERMEDIATE_POS = new ShootingParameters(35, 90);
  public static final ShootingParameters AGAINST_ENEMY = new ShootingParameters(75, 48.0);
  public static final ShootingParameters PODIUM_SHOOT = new ShootingParameters(75, 44.5);
  public static final ShootingParameters FAR_SHOOT = new ShootingParameters(75, 26.2);
  public static final ShootingParameters NEAR_SHOOT = new ShootingParameters(65, 26.2);
  public static final ShootingParameters TRAP = new ShootingParameters(60, 62);

  // everything above has not been fine tuned for new shooter !TODO
  public static final ShootingParameters IN_FRONT_OF_WING = new ShootingParameters(75, 34);

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
