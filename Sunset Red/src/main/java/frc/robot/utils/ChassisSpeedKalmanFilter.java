package frc.robot.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;

/*
 * Filter Drivetrain Speed through a Kalman Filter
 */
public class ChassisSpeedKalmanFilter {

  private static Vector<N1> u = VecBuilder.fill(0.0);

  private KalmanFilter<N3, N1, N3> mChassisSpeedFilter;
  private double mDt;

  public ChassisSpeedKalmanFilter(double xyDev, double angleDev, double dt) {
    mDt = dt;
    mChassisSpeedFilter =
        new KalmanFilter<>(
            Nat.N3(),
            Nat.N3(),
            // system is given as continuous
            new LinearSystem<N3, N1, N3>(
                // mat A (state -> state change) = 0
                MatBuilder.fill(Nat.N3(), Nat.N3(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                // mat B (input -> state change) = 0
                VecBuilder.fill(0.0, 0.0, 0.0),
                // mat C (state -> output) = I
                MatBuilder.fill(Nat.N3(), Nat.N3(), 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                // mat D (input -> output) = 0
                VecBuilder.fill(0.0, 0.0, 0.0)),
            // model state stdDev (Q) = 0
            VecBuilder.fill(0.0, 0.0, 0.0),
            // measurement stdDev (R)
            VecBuilder.fill(xyDev, xyDev, angleDev),
            dt);
  }

  public ChassisSpeeds correctAndPredict(ChassisSpeeds kinematic_speed) {
    // filtering
    mChassisSpeedFilter.predict(u, mDt);
    mChassisSpeedFilter.correct(
        u,
        VecBuilder.fill(
            kinematic_speed.vxMetersPerSecond,
            kinematic_speed.vyMetersPerSecond,
            kinematic_speed.omegaRadiansPerSecond));
    return new ChassisSpeeds(
        mChassisSpeedFilter.getXhat(0),
        mChassisSpeedFilter.getXhat(1),
        mChassisSpeedFilter.getXhat(2));
  }
}
