package frc.robot.commands;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;
import java.util.Objects;
import java.util.function.Supplier;

public class PepGuardiolaCommand extends Command {

  // scalar when operator adjusts speed/angle
  private static final double kManualSpeedOffsetScalar = 5.0;
  private static final double kManualHeadingOffsetScalar = Math.toRadians(4.0);

  public enum GoalZone {
    UP(new Translation2d(9.5, 1.4)),
    DOWN(new Translation2d(1.2, 7.0)),
    LEFT(new Translation2d(6.4, 7.6)),
    RIGHT(new Translation2d(6.4, 4.0));

    // goal position in (blue-origin) field space
    // assuming we are blue alliance (NOTE THAT IN PPT WE ARE RED)
    public final Translation2d target;

    private GoalZone(Translation2d target) {
      this.target = target;
    }
  }

  // the param for low shoot
  private static final ShootingParameters kLowShootParam = new ShootingParameters(40.0, 30.0);

  // the speed for high shoot
  private static final InterpolatingDoubleTreeMap kHighShootSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    // TODO : TUNE
    // distance meters <-> shooter speed rps
    // the distance here is the distance from the robot to the landing position of note
    // lowest allowable speed is ~10, lower then that will stuck note
    // typical distance roughly 6-11m
    kHighShootSpeedMap.put(11.35, 67.0);
    kHighShootSpeedMap.put(11.25, 66.0);
    kHighShootSpeedMap.put(11.05, 68.0);
    kHighShootSpeedMap.put(10.85, 66.0);
    kHighShootSpeedMap.put(10.55, 63.0);

    kHighShootSpeedMap.put(10.25, 64.0);
    kHighShootSpeedMap.put(9.75, 59.0);
    kHighShootSpeedMap.put(9.45, 57.0);
    kHighShootSpeedMap.put(8.95, 55.0);
    kHighShootSpeedMap.put(8.45, 53.0);

    kHighShootSpeedMap.put(8.25, 53.0);

    kHighShootSpeedMap.put(7.9, 54.0);
    kHighShootSpeedMap.put(5.9, 45.0);
    kHighShootSpeedMap.put(5.4, 43.0);
  }

  // the arm angle for high shoot
  private static final InterpolatingDoubleTreeMap kHighShootAngleMap =
      new InterpolatingDoubleTreeMap();

  static {
    // TODO : TUNE
    // distance meters <-> arm angle degrees
    // the distance here is the distance from the robot to the landing position of note
    // lowest allowable angle is 28, lower then that will hit camera
    // typical distance roughly 6-11m
    kHighShootAngleMap.put(11.35, 45.0);

    kHighShootAngleMap.put(11.25, 46.0);
    kHighShootAngleMap.put(11.05, 50.0);
    kHighShootAngleMap.put(10.85, 50.0);
    kHighShootAngleMap.put(10.55, 50.0);
    kHighShootAngleMap.put(10.25, 50.0);
    kHighShootAngleMap.put(9.75, 50.0);
    kHighShootAngleMap.put(9.45, 50.0);
    kHighShootAngleMap.put(8.95, 50.0);
    kHighShootAngleMap.put(8.45, 53.0);
    kHighShootAngleMap.put(8.25, 54.0);
    kHighShootAngleMap.put(7.9, 54.0);
    kHighShootAngleMap.put(5.9, 54.0);
    kHighShootAngleMap.put(5.4, 54.0);
  }

  private DrivetrainSubsystem sDrivetrainSubsystem;
  private Arm sArm;
  private Transfer sTransfer;
  private Shooter sShooter;
  private Intake sIntake;
  private Supplier<Translation2d> mDriveVectorSupplier;
  private GoalZone mGoalZone;
  private Supplier<Double> mSpeedOffsetSupplier;
  private Supplier<Double> mHeadingOffsetSupplier;

  private final ProfiledPIDController mProfiledPID =
      new ProfiledPIDController(
          4.0, 0, 0.0, new TrapezoidProfile.Constraints(Math.toRadians(540), Math.toRadians(720)));

  private enum DeliverState {
    AIMING,
    FEEDING,
    END
  }

  private DeliverState mDeliverState = DeliverState.AIMING;

  double shooterSpeed, armAngle;

  private static final double READY_TO_FEED_DELAY = 0.2;
  private DualEdgeDelayedBoolean mReadyToFeed;

  private Timer stateTimer = new Timer();

  public PepGuardiolaCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Transfer transfer,
      Shooter shooter,
      Intake intake,
      Supplier<Translation2d> driveVectorSupplier,
      GoalZone goalZone,
      Supplier<Double> speedOffsetSupplier,
      Supplier<Double> headingOffsetSupplier) {
    sDrivetrainSubsystem = drivetrainSubsystem;
    sArm = arm;
    sTransfer = transfer;
    sShooter = shooter;
    sIntake = intake;
    mDriveVectorSupplier = driveVectorSupplier;
    mGoalZone = Objects.requireNonNull(goalZone);
    mSpeedOffsetSupplier = speedOffsetSupplier;
    mHeadingOffsetSupplier = headingOffsetSupplier;
    addRequirements(drivetrainSubsystem, arm, transfer, shooter, intake);
  }

  @Override
  public void initialize() {
    mDeliverState = DeliverState.AIMING;

    mProfiledPID.reset(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
    mProfiledPID.setGoal(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.setTolerance(Math.toRadians(1.0));

    mReadyToFeed =
        new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), READY_TO_FEED_DELAY, EdgeType.RISING);
    stateTimer.reset();

    sTransfer.stop();
  }

  @Override
  public void execute() {
    if (!sTransfer.isOmronDetected()) {
      sIntake.setIntake();
    } else {
      sIntake.stop();
    }
    switch (mDeliverState) {
      case AIMING:
        handleAiming();
        break;
      case FEEDING:
        handleFeeding();
      case END:
      default:
        break;
    }
  }

  private void handleAiming() {
    // calculate distance
    Translation2d targetVector = calculateTarget();
    Rotation2d targetRobotHeading = targetVector.getAngle().rotateBy(Rotation2d.fromDegrees(180.0));
    double targetDist = targetVector.getNorm();

    // find shoot parameter
    boolean isLowShoot = checkLowShoot();
    if (isLowShoot) {
      shooterSpeed = kLowShootParam.speed_rps;
      armAngle = kLowShootParam.angle_deg;
    } else {
      shooterSpeed = kHighShootSpeedMap.get(targetDist) + mSpeedOffsetSupplier.get() * kManualSpeedOffsetScalar;
      armAngle = kHighShootAngleMap.get(targetDist);
    }

    // run subsystem
    double headingGoal = targetRobotHeading.getRadians() + mHeadingOffsetSupplier.get() * kManualHeadingOffsetScalar;
    mProfiledPID.setGoal(headingGoal);
    // in rad/s
    double turnspeed =
        mProfiledPID.calculate(sDrivetrainSubsystem.getHeading().getRadians())
            + mProfiledPID.getSetpoint().velocity;
    if (mProfiledPID.atGoal()) {
      turnspeed = 0.0;
    }
    sDrivetrainSubsystem.drive(mDriveVectorSupplier.get(), turnspeed, true);
    sArm.setAngle(armAngle);
    sShooter.setTargetVelocity(shooterSpeed);

    // detect time to feed
    if (decideToFeed()) {
      mDeliverState = DeliverState.FEEDING;
      sDrivetrainSubsystem.drive(mDriveVectorSupplier.get(), turnspeed, true);
      stateTimer.start();
    }
  }

  private static final double kWingDeadlineX = 10.1;

  private boolean decideToFeed() {
    // find if zone readly feed
    boolean zoneOk = true;
    if (mGoalZone == GoalZone.DOWN) {
      // you need to cross wing to be ok
      zoneOk = isOutsideOppositeWing();
    }

    // find if superstruct ok
    boolean shootOk =
        Math.abs(sShooter.getAverageVelocity() - shooterSpeed) < 3.0
            && Math.abs(sArm.getAngleDeg() - armAngle) < 3.0
            && Math.abs(
                    sDrivetrainSubsystem
                        .getHeading()
                        .minus(new Rotation2d(mProfiledPID.getGoal().position))
                        .getDegrees())
                < 5.0;

    // return shootOk&&zoneOk;
    return mReadyToFeed.update(Timer.getFPGATimestamp(), shootOk && zoneOk);
  }

  private boolean checkLowShoot() {
    if (mGoalZone == GoalZone.DOWN && inLeftRightField()) {
      // leftright field low-shoot to down zone
      return true;
    }

    // default high shoot
    return false;
  }

  private static final double kLeftRightLineY = 6.0;

  private boolean inLeftRightField() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d robotPose = sDrivetrainSubsystem.getPose();
    if (alliance == Alliance.Blue) {
      return robotPose.getX() < 8.27 && robotPose.getY() > kLeftRightLineY;
    } else {
      return robotPose.getX() > 8.27 && robotPose.getY() > kLeftRightLineY;
    }
  }

  public boolean isOutsideOppositeWing() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d robotPose = sDrivetrainSubsystem.getPose();
    if (alliance == Alliance.Blue) {
      return robotPose.getX() < kWingDeadlineX;
    } else {
      return robotPose.getX() > 16.54 - kWingDeadlineX;
    }
  }

  private static final double kFrontLineX = 13.5;

  private boolean inFrontField() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double robotX = sDrivetrainSubsystem.getPose().getX();
    if (alliance == Alliance.Blue) {
      return robotX > kFrontLineX;
    } else {
      return robotX < 16.54 - kFrontLineX;
    }
  }

  private Translation2d calculateTarget() {
    // find goal by alliance
    Translation2d goal;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      goal = mGoalZone.target;
    } else { // red
      if (mGoalZone == GoalZone.UP) {
        goal = GeometryUtil.flipFieldPosition(GoalZone.UP.target);
      } else if (mGoalZone == GoalZone.DOWN) {
        goal = GeometryUtil.flipFieldPosition(GoalZone.DOWN.target);
      } else if (mGoalZone == GoalZone.LEFT) {
        // THIS IS NOT A TYPO: in blue/red the left and right reverted.
        goal = GeometryUtil.flipFieldPosition(GoalZone.RIGHT.target);
      } else { // if (mGoalZone == GoalZone.RIGHT) {
        // THIS IS NOT A TYPO: in blue/red the left and right reverted.
        goal = GeometryUtil.flipFieldPosition(GoalZone.LEFT.target);
      }
    }

    // find robotToGoal
    return goal.minus(sDrivetrainSubsystem.getPose().getTranslation());
  }

  private void handleFeeding() {
    sTransfer.setVoltage(Transfer.FEED_VOLTS);
    sDrivetrainSubsystem.drive(mDriveVectorSupplier.get(), 0, true);

    if (stateTimer.hasElapsed(0.5)) {
      // restart
      mDeliverState = DeliverState.AIMING;
    }
  }

  @Override
  public void end(boolean interrupted) {
    sDrivetrainSubsystem.stop();
    sArm.stop();
    sShooter.stop();
    sTransfer.stop();
    sIntake.stop();
    stateTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelIncoming;
  }
}
