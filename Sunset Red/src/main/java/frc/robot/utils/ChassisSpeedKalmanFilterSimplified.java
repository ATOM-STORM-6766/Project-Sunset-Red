package frc.robot.utils;

import edu.wpi.first.math.DARE;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/*
 * Filter Drivetrain Speed through a Kalman Filter
 * Simplified from original Kalman Filter because
 * we only do A = 0, C = I which a lot of math can be very easy
 */
public class ChassisSpeedKalmanFilterSimplified {

  private Matrix<N3, N3> mC;
  private Matrix<N3, N3> mDiscQ;
  private Matrix<N3, N3> mDiscR;
  private Matrix<N3, N3> mK;
  private Matrix<N3, N3> mP;
  private Matrix<N3, N1> mXHat;

  public ChassisSpeedKalmanFilterSimplified(double xyDev, double angleDev, double dt) {
    // state model deviation (Q) = 0
    Vector<N3> stateStdDev = VecBuilder.fill(10.0, 10.0, 10.0);
    // measurement deviation (R)
    Vector<N3> measurementStdDev = VecBuilder.fill(xyDev, xyDev, angleDev);

    Matrix<N3, N3> mContQ = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), stateStdDev);
    Matrix<N3, N3> mContR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), measurementStdDev);

    Matrix<N3, N3> mDiscA = Matrix.eye(Nat.N3());
    mDiscQ = mContQ.times(dt);
    mDiscR = mContR.div(dt);
    mC = Matrix.eye(Nat.N3());

    // discrete time algebraic Riccati equation (DARE)
    mP = new Matrix<>(DARE.dare(mDiscA.transpose(), mC.transpose(), mDiscQ, mDiscR));
    mXHat = new Matrix<>(Nat.N3(), Nat.N1());

    // Kalman Gain (K)
    mK = new Matrix<>(Nat.N3(), Nat.N3());
  }

  public ChassisSpeeds correctAndPredict(ChassisSpeeds kinematic_speed) {
    Vector<N3> vy =
        VecBuilder.fill(
            kinematic_speed.vxMetersPerSecond,
            kinematic_speed.vyMetersPerSecond,
            kinematic_speed.omegaRadiansPerSecond);
    // predict
    // 1. m_xHat = mDiscA @ m_xHat + mDiscB @ u (skip)
    // 2. Pₖ₊₁⁻ = APₖ⁻Aᵀ + Q
    mP = mP.plus(mDiscQ);
    // correct
    // 1. K = PCᵀ(CPCᵀ+R)⁻¹
    for (int i = 0; i < 3; i++) {
      mK.set(i, i, mP.get(i, i) / (mP.get(i, i) + mDiscR.get(i, i)));
    }
    // 2. x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − (Cx̂ₖ₊₁⁻ + Duₖ₊₁))
    // mXHat = mXHat.plus(mK.times(vy.minus(mC.times(mXHat))));
    for (int i = 0; i < 3; i++) {
      double y = vy.get(i, 0);
      double x = mXHat.get(i, 0);
      double k = mK.get(i, i);
      x = x + k * (y - x);
      mXHat.set(i, 0, x);
    }
    // 3. Pₖ₊₁⁺ = (I−Kₖ₊₁C)Pₖ₊₁⁻(I−Kₖ₊₁C)ᵀ + Kₖ₊₁RKₖ₊₁ᵀ
    for (int i = 0; i < 3; i++) {
      double k = mK.get(i, i);
      double p = mP.get(i, i);
      double r = mDiscR.get(i, i);
      p = Math.pow(1 - k, 2.0) * p + Math.pow(k, 2.0) * r;
      mP.set(i, i, p);
    }
    return new ChassisSpeeds(mXHat.get(0, 0), mXHat.get(1, 0), mXHat.get(2, 0));
  }
}
