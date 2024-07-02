package frc.robot.lib6907.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all of the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 *
 * <p>Source:
 * https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/swerve/SwerveSetpointGenerator.java
 * Mitigated to WPILIB by TonyBest
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics mKinematics;

  public SwerveSetpointGenerator(final SwerveDriveKinematics kinematics) {
    this.mKinematics = kinematics;
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  private boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  private interface Function2d {
    public double f(double x, double y);
  }

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || epsilonEquals(f_0, f_1, 1e-12)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation,
      int max_iterations) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func =
        (x, y) -> {
          return unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_vel_step,
      int max_iterations) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func =
        (x, y) -> {
          return Math.hypot(x, y) - offset;
        };
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS(
      double x_0, double y_0, double x_1, double y_1, double max_vel_step) {
    // Our drive velocity between s=0 and s=1 is quadratic in s:
    // v^2 = ((x_1 - x_0) * s + x_0)^2 + ((y_1 - y_0) * s + y_0)^2
    //     = a * s^2 + b * s + c
    // Where:
    //   a = (x_1 - x_0)^2 + (y_1 - y_0)^2
    //   b = 2 * x_0 * (x_1 - x_0) + 2 * y_0 * (y_1 - y_0)
    //   c = x_0^2 + y_0^2
    // We want to find where this quadratic results in a velocity that is > max_vel_step from our
    // velocity at s=0:
    // sqrt(x_0^2 + y_0^2) +/- max_vel_step = ...quadratic...
    final double dx = x_1 - x_0;
    final double dy = y_1 - y_0;
    final double a = dx * dx + dy * dy;
    final double b = 2.0 * x_0 * dx + 2.0 * y_0 * dy;
    final double c = x_0 * x_0 + y_0 * y_0;
    final double v_limit_upper_2 = Math.pow(Math.hypot(x_0, y_0) + max_vel_step, 2.0);
    final double v_limit_lower_2 = Math.pow(Math.hypot(x_0, y_0) - max_vel_step, 2.0);
    return 0.0;
  }

  private Twist2d csToTwist2d(ChassisSpeeds cs) {
    return new Twist2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond);
  }

  private boolean epsilonEquals(double a, double b, double eps) {
    return (a - eps <= b) && (a + eps >= b);
  }

  private boolean epsilonEquals(Twist2d a, Twist2d b, double eps) {
    return epsilonEquals(a.dx, b.dx, eps)
        && epsilonEquals(a.dy, b.dy, eps)
        && epsilonEquals(a.dtheta, b.dtheta, eps);
  }

  private Rotation2d flip(Rotation2d angle) {
    if (!Double.isNaN(angle.getSin()) && !Double.isNaN(angle.getCos())) {
      return new Rotation2d(-angle.getCos(), -angle.getSin());
    } else {
      return Rotation2d.fromRadians(angle.getRadians() + Math.PI);
    }
  }

  /**
   * Generate a new setpoint.
   *
   * @param limits The kinematic limits to respect for this setpoint.
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredState The desired state of motion, such as from the driver sticks or a path
   *     following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all of the KinematicLimits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveKinematicLimits limits,
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredState,
      double dt,
      DrivetrainSubsystem swerve) {
    final Translation2d[] modules =
        Arrays.stream(swerve.getModuleArray())
            .map(m -> m.getModulePositionToCenter())
            .toArray(size -> new Translation2d[size]);
    final Twist2d kTwist2dIdentity = new Twist2d();
    final double kEpsilon = 1e-12;
    final Rotation2d kRotation2dPI = Rotation2d.fromRadians(Math.PI);
    final Rotation2d kRotation2dIdentity = Rotation2d.fromRadians(0.0);

    SwerveModuleState[] desiredModuleState = mKinematics.toSwerveModuleStates(desiredState);
    // Make sure desiredState respects velocity limits.
    if (limits.kMaxDriveVelocity > 0.0) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.kMaxDriveVelocity);
      desiredState = mKinematics.toChassisSpeeds(desiredModuleState);
    }

    // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
    // just use the previous angle.
    // override: GO X CROSS IF DESIRE A STOP
    boolean need_to_steer = true;
    if (epsilonEquals(csToTwist2d(desiredState), kTwist2dIdentity, kEpsilon)) {
      if (epsilonEquals(csToTwist2d(prevSetpoint.mChassisSpeeds), kTwist2dIdentity, kEpsilon)) {
        // from previous stop to stop, go X CROSS
        need_to_steer = false;
        for (int i = 0; i < 4; i++) {
          Rotation2d targ = modules[i].getAngle();
          Rotation2d diff = prevSetpoint.mModuleStates[i].angle.minus(targ);
          if (flipHeading(diff)) {
            targ = flip(targ);
            diff = prevSetpoint.mModuleStates[i].angle.minus(targ);
          }
          if (!diff.equals(kRotation2dIdentity)) {
            need_to_steer = true;
          }
          desiredModuleState[i].angle = targ;
          desiredModuleState[i].speedMetersPerSecond = 0.0;
        }
      } else {
        // from previously non stop to full stop, keep angle
        need_to_steer = false;
        for (int i = 0; i < 4; ++i) {
          desiredModuleState[i].angle = prevSetpoint.mModuleStates[i].angle;
          desiredModuleState[i].speedMetersPerSecond = 0.0;
        }
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[4];
    double[] prev_vy = new double[4];
    Rotation2d[] prev_heading = new Rotation2d[4];
    double[] desired_vx = new double[4];
    double[] desired_vy = new double[4];
    Rotation2d[] desired_heading = new Rotation2d[4];
    boolean all_modules_should_flip = true;
    for (int i = 0; i < 4; ++i) {
      prev_vx[i] =
          prevSetpoint.mModuleStates[i].angle.getCos()
              * prevSetpoint.mModuleStates[i].speedMetersPerSecond;
      prev_vy[i] =
          prevSetpoint.mModuleStates[i].angle.getSin()
              * prevSetpoint.mModuleStates[i].speedMetersPerSecond;
      prev_heading[i] = prevSetpoint.mModuleStates[i].angle;
      if (prevSetpoint.mModuleStates[i].speedMetersPerSecond < 0.0) {
        prev_heading[i] = flip(prev_heading[i]);
      }
      desired_vx[i] =
          desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
      desired_vy[i] =
          desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
      desired_heading[i] = desiredModuleState[i].angle;
      if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
        desired_heading[i] = flip(desired_heading[i]);
      }
      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }
    if (all_modules_should_flip
        && !epsilonEquals(csToTwist2d(prevSetpoint.mChassisSpeeds), kTwist2dIdentity, kEpsilon)
        && !epsilonEquals(csToTwist2d(desiredState), kTwist2dIdentity, kEpsilon)) {
      // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
      // of the desired
      // angle, and accelerate again.
      return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt, swerve);
    }

    // Compute the deltas between start and goal. We can then interpolate from the start state to
    // the goal state; then
    // find the amount we can move from start towards goal in this cycle such that no kinematic
    // limit is exceeded.
    double dx = desiredState.vxMetersPerSecond - prevSetpoint.mChassisSpeeds.vxMetersPerSecond;
    double dy = desiredState.vyMetersPerSecond - prevSetpoint.mChassisSpeeds.vyMetersPerSecond;
    double dtheta =
        desiredState.omegaRadiansPerSecond - prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond;

    // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
    // desiredState.
    double min_s = 1.0;

    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since
    // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(4);
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle,
    // and then backing out the maximum interpolant between start and goal states. We remember the
    // minimum across all modules, since
    // that is the active constraint.
    final double max_theta_step = dt * limits.kMaxSteeringVelocity;
    for (int i = 0; i < 4; ++i) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.mModuleStates[i].angle));
        continue;
      }
      overrideSteering.add(Optional.empty());
      if (epsilonEquals(prevSetpoint.mModuleStates[i].speedMetersPerSecond, 0.0, kEpsilon)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based
        // purely on rotation in place.

        var necessaryRotation =
            prevSetpoint.mModuleStates[i].angle.unaryMinus().rotateBy(desiredModuleState[i].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(kRotation2dPI);
        }
        // getRadians() bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
          // Don't limit the global min_s;
          continue;
        } else {
          // Adjust steering by max_theta_step.
          overrideSteering.set(
              i,
              Optional.of(
                  prevSetpoint.mModuleStates[i].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          min_s = 0.0;
          continue;
        }
      }
      if (min_s == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      final int kMaxIterations = 8;
      double s =
          findSteeringMaxS(
              prev_vx[i],
              prev_vy[i],
              prev_heading[i].getRadians(),
              desired_vx[i],
              desired_vy[i],
              desired_heading[i].getRadians(),
              max_theta_step,
              kMaxIterations);
      min_s = Math.min(min_s, s);
    }

    // Enforce drive wheel acceleration limits.
    final double max_vel_step = dt * limits.kMaxDriveAcceleration;
    for (int i = 0; i < 4; ++i) {
      if (min_s == 0.0) {
        // No need to carry on.
        break;
      }
      double vx_min_s =
          min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
      double vy_min_s =
          min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster
      // than that.
      final int kMaxIterations = 10;
      double s =
          min_s
              * findDriveMaxS(
                  prev_vx[i],
                  prev_vy[i],
                  Math.hypot(prev_vx[i], prev_vy[i]),
                  vx_min_s,
                  vy_min_s,
                  Math.hypot(vx_min_s, vy_min_s),
                  max_vel_step,
                  kMaxIterations);
      min_s = Math.min(min_s, s);
    }

    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.mChassisSpeeds.vxMetersPerSecond + min_s * dx,
            prevSetpoint.mChassisSpeeds.vyMetersPerSecond + min_s * dy,
            prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
    var retStates = mKinematics.toSwerveModuleStates(retSpeeds);
    for (int i = 0; i < 4; ++i) {
      final var maybeOverride = overrideSteering.get(i);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
          retStates[i].speedMetersPerSecond *= -1.0;
        }
        retStates[i].angle = override;
      }
      final var deltaRotation =
          prevSetpoint.mModuleStates[i].angle.unaryMinus().rotateBy(retStates[i].angle);
      if (flipHeading(deltaRotation)) {
        retStates[i].angle = flip(retStates[i].angle);
        retStates[i].speedMetersPerSecond *= -1.0;
      }
    }
    return new SwerveSetpoint(retSpeeds, retStates);
  }
}
