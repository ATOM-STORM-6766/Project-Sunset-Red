package frc.robot.lib6907.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
  public ChassisSpeeds mChassisSpeeds;
  public SwerveModuleState[] mModuleStates;

  public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
    this.mChassisSpeeds = chassisSpeeds;
    this.mModuleStates = initialStates;
  }

  @Override
  public String toString() {
    String ret = mChassisSpeeds.toString() + "\n";
    for (int i = 0; i < mModuleStates.length; ++i) {
      ret += "  " + mModuleStates[i].toString() + "\n";
    }
    return ret;
  }
}
