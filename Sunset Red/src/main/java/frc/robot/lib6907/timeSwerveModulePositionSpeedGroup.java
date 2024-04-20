package frc.robot.lib6907;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class timeSwerveModulePositionSpeedGroup {
  public final SwerveModulePosition modulePosition;
  public final SwerveModuleState moduleState;
  public final double timestamp;

  public timeSwerveModulePositionSpeedGroup(
      SwerveModulePosition modulePosition, SwerveModuleState moduleState, double timestamp) {
    this.modulePosition = modulePosition;
    this.moduleState = moduleState;
    this.timestamp = timestamp;
  }
}
