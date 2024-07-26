package frc.robot.commands;

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
import java.util.function.Supplier;
import java.util.Objects;
import com.pathplanner.lib.util.GeometryUtil;

public class PepGuardiolaCommand extends Command {

  public enum GoalZone {
    UP(new Translation2d(15.1, 1.5)),
    DOWN(new Translation2d(1.7, 7.4)),
    LEFT(new Translation2d(7.2, 6.7)),
    RIGHT(new Translation2d(7.2, 4.1));

    // goal position in (blue-origin) field space
    // assuming we are blue alliance (NOTE THAT IN PPT WE ARE RED)
    public final Translation2d target;
    private GoalZone(Translation2d target) {
      this.target = target;
    }
  }

  // the param for low shoot
  private static final ShootingParameters kLowShootParam = new ShootingParameters(60.0, 30.0);

  // the speed for high shoot
  private static final double kHighShootSpeed = 65.0;

  // the arm angle for high shoot
  private static final InterpolatingDoubleTreeMap kHighShootAngleMap = new InterpolatingDoubleTreeMap();
  static {
    // TODO : TUNE
    // distance meters <-> arm angle degrees
    // the distance here is the distance from the robot to the landing position of note
    // lowest allowable angle is 28, lower then that will hit camera
    // typical distance roughly 6-10m
    kHighShootAngleMap.put(6.0, 60.0);
    kHighShootAngleMap.put(10.0, 45.0);
  }

  private DrivetrainSubsystem sDrivetrainSubsystem;
  private Arm sArm;
  private Transfer sTransfer;
  private Shooter sShooter;
  private Supplier<Translation2d> mDriveVectorSupplier;
  private GoalZone mGoalZone;
  
  private final ProfiledPIDController mProfiledPID =
      new ProfiledPIDController(
          4.0, 0, 0.0, new TrapezoidProfile.Constraints(Math.toRadians(540), Math.toRadians(720)));


  private enum DeliverState {
    AIMING, FEEDING, END
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
      Supplier<Translation2d> driveVectorSupplier,
      GoalZone goalZone) {
    sDrivetrainSubsystem = drivetrainSubsystem;
    sArm = arm;
    sTransfer = transfer;
    sShooter = shooter;
    mDriveVectorSupplier = driveVectorSupplier;
    mGoalZone = Objects.requireNonNull(goalZone);

    addRequirements(drivetrainSubsystem, arm, transfer, shooter);
  }

  @Override
  public void initialize() {
    mDeliverState = DeliverState.AIMING;
    
    mProfiledPID.reset(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
    mProfiledPID.setGoal(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.setTolerance(Math.toRadians(3.0));

    mReadyToFeed = new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 
      READY_TO_FEED_DELAY, EdgeType.RISING);
    stateTimer.reset();

    sTransfer.stop();
  }

  @Override
  public void execute() {
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
      shooterSpeed = kHighShootSpeed;
      armAngle = kHighShootAngleMap.get(targetDist);
    }

    // run subsystem
    mProfiledPID.setGoal(targetRobotHeading.getRadians());
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
      stateTimer.start();
    }
  }

  private static final double kWingDeadlineX = 10.1;
  private boolean decideToFeed() {
    // find if zone readly feed
    boolean zoneOk = true;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d robotPose = sDrivetrainSubsystem.getPose();
    if (mGoalZone == GoalZone.DOWN) {
      // you need to cross wing to be ok
      if (alliance == Alliance.Blue) {
        zoneOk = robotPose.getX() < kWingDeadlineX;
      } else {
        zoneOk = robotPose.getX() > 16.54 - kWingDeadlineX;
      }
    }
    
    // find if superstruct ok
    boolean shootOk = Math.abs(sShooter.getAverageVelocity() - shooterSpeed) < 3.0
      && Math.abs(sArm.getAngleDeg() - armAngle) < 2.0
      && mProfiledPID.atGoal();
    
    return mReadyToFeed.update(Timer.getFPGATimestamp(), shootOk && zoneOk);
  }

  private boolean checkLowShoot() {
    if (mGoalZone == GoalZone.UP && inFrontField()) {
      // front field low-shoot to up zone
      return true;
    }
    if (mGoalZone == GoalZone.DOWN && inLeftRightField()) {
      // leftright field low-shoot to down zone
      return true;
    }

    // default high shoot
    return false;
  }

  private static final double kLeftRightLineY = 3.0;
  private boolean inLeftRightField() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d robotPose = sDrivetrainSubsystem.getPose();
    if (alliance == Alliance.Blue) {
      return robotPose.getX() < 8.27 && robotPose.getY() > kLeftRightLineY;
    } else {
      return robotPose.getX() > 8.27 && robotPose.getY() > kLeftRightLineY;
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
    if (stateTimer.hasElapsed(0.5)) {
      mDeliverState = DeliverState.END;
    }
  }

  @Override
  public void end(boolean interrupted) {
    sDrivetrainSubsystem.stop();
    sArm.stop();
    sShooter.stop();
    sTransfer.stop();
    stateTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return mDeliverState == DeliverState.END;
  }
}
