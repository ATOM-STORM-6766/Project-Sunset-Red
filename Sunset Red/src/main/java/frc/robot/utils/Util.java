package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

public class Util {

  public static void checkReturn(String key, StatusCode code) {
    if (!code.isOK()) {
      DriverStation.reportError(
          String.format("[%s] %s: %s", key, code.getName(), code.getDescription()), null);
    }
  }
}
