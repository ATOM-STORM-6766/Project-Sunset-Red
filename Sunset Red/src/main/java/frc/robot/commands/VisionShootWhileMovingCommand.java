package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionShootConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.*;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionShootWhileMovingCommand extends Command {

  public enum AimingMode {
    PNP3D, PITCH2D
  }

  private static final double kHeadingOffsetScalarDeg = 8.0;
  private static final double kAngleOffsetScalarDeg = 5.0;
  // the aim offset scalar when you are aside from the speaker
  private static final double kFarCornerScalar = 0.0;

  private DrivetrainSubsystem sDrivetrainSubsystem;
  private Arm sArm;
  private Transfer sTransfer;
  private Shooter sShooter;
  private Intake sIntake;
  private Supplier<Translation2d> mDriveVectorSupplier;

  private final PIDController mHeadingPID = new PIDController(5.0, 0, 0.2);

  private enum State {
    AIMING, FEEDING, END
  }
  private State mState = State.AIMING;

  double shooterSpeed, armAngle;

  private static final double READY_TO_FEED_DELAY = 0.1;
  private DualEdgeDelayedBoolean mReadyToFeed;

  private Supplier<Double> mHeadingOffsetSupplier;
  private Supplier<Double> mAngleOffsetSupplier;
  private AimingMode mAimingMode;

  private Timer stateTimer = new Timer();

  public VisionShootWhileMovingCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Transfer transfer,
      Shooter shooter,
      Intake intake,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Double> headingOffsetSupplier,
      Supplier<Double> angleOffsetSupplier,
      AimingMode aimingMode) {
    sDrivetrainSubsystem = drivetrainSubsystem;
    sArm = arm;
    sTransfer = transfer;
    sShooter = shooter;
    sIntake = intake;
    mDriveVectorSupplier = driveVectorSupplier;
    mHeadingOffsetSupplier = headingOffsetSupplier;
    mAngleOffsetSupplier = angleOffsetSupplier;
    mAimingMode = aimingMode;

    addRequirements(drivetrainSubsystem, arm, transfer, shooter, intake);
  }

  @Override
  public void initialize() {
    mState = State.AIMING;

    mHeadingPID.reset();
    mHeadingPID.enableContinuousInput(-Math.PI, Math.PI);
    mHeadingPID.setSetpoint(sDrivetrainSubsystem.getHeading().getRadians());
    mHeadingPID.setTolerance(Math.toRadians(3.0));

    mReadyToFeed = new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 
      READY_TO_FEED_DELAY, EdgeType.RISING);
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
    switch (mState) {
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
    Translation2d driveVector = mDriveVectorSupplier.get();
    // calculate distance
    Translation2d targetVector;
    if (mAimingMode == AimingMode.PNP3D) {
      targetVector = calculateTarget3D(driveVector);
    } else {
      targetVector = calculateTarget2D(driveVector);
      if (targetVector == null) {
        // there is chance that 2d tag cant be seen
        mState = State.END;
        return;
      }
    }
    Rotation2d targetRobotHeading = targetVector.getAngle().rotateBy(Rotation2d.fromDegrees(180.0));
    // far corner offset
    Rotation2d entryAngleRev;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      entryAngleRev = Rotation2d.fromDegrees(0.0);
    } else { // red
      entryAngleRev = Rotation2d.fromDegrees(180.0);
    }
    Rotation2d farShootOffset = targetRobotHeading.minus(entryAngleRev).times(kFarCornerScalar);
    targetRobotHeading = targetRobotHeading.plus(farShootOffset);
    targetRobotHeading = targetRobotHeading.plus(Rotation2d.fromDegrees(kHeadingOffsetScalarDeg * mHeadingOffsetSupplier.get()));
    double targetDist = targetVector.getNorm();

    // find shoot parameter
    shooterSpeed = VisionShootConstants.kSpeakerRPSMap.get(targetDist);
    armAngle = VisionShootConstants.kSpeakerAngleMap.get(targetDist) + kAngleOffsetScalarDeg * mAngleOffsetSupplier.get();

    // run subsystem
    // in rad/s
    double turnspeed =
        mHeadingPID.calculate(sDrivetrainSubsystem.getHeading().getRadians(), targetRobotHeading.getRadians());
    if (mHeadingPID.atSetpoint()) {
      turnspeed = 0.0;
    }
    sDrivetrainSubsystem.drive(driveVector, turnspeed, true);
    sArm.setAngle(armAngle);
    sShooter.setTargetVelocity(shooterSpeed);
    
    // detect time to feed
    if (decideToFeed()) {
      mState = State.FEEDING;
      stateTimer.start();
    }
  }

  private Translation2d calculateTarget2D(Translation2d driveVector) {
    int tagId = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 7 : 4;
    var tagresult = ApriltagCoprocessor.getInstance().getShooterSideLastResult();
    if (!tagresult.hasTargets()) {
      SmartDashboard.putString("Vision Shoot 2D", "NO_TAG_SEEN");
      return null;
    }

    PhotonTrackedTarget targetOfInterest = null;
    for (var tgt : tagresult.getTargets()) {
      if (tgt.getFiducialId() == tagId) {
        targetOfInterest = tgt;
        break;
      }
    }
    if (targetOfInterest == null) {
      SmartDashboard.putString("Vision Shoot 2D", "NO_SPEAKER_TAG");
      return null;
    }
    
    final Transform3d camExtrinsic = ApriltagCoprocessor.getInstance().kRobotToCameraForShooterSide;

    // LETS JUST DO SOME WILD CALCULATION
    // TODO : CHECK DIRECTION
    final double kTagHeight = 57.13 * 0.0254;
    Rotation2d targetHeading = sDrivetrainSubsystem.getHeading(tagresult.getTimestampSeconds())
      .plus(Rotation2d.fromDegrees(targetOfInterest.getYaw()))
      .plus(Rotation2d.fromDegrees(180.0));
    double targetDist = (kTagHeight - camExtrinsic.getZ()) / Math.tan(
      Math.abs(camExtrinsic.getRotation().getY()) + Math.toRadians(targetOfInterest.getPitch())) + Math.abs(camExtrinsic.getX());
    SmartDashboard.putString("Vision Shoot 2D", "OK");
    Translation2d ret = new Translation2d(targetDist, targetHeading);
    SmartDashboard.putString("Vision Shoot 2D Vector", ret.toString());
    return ret;
  }

  private boolean decideToFeed() {
    // find if superstruct ok
    boolean shootOk = Math.abs(sShooter.getAverageVelocity() - shooterSpeed) < 2.0
      && Math.abs(sArm.getAngleDeg() - armAngle) < 2.0
      && Math.abs(mHeadingPID.getSetpoint() - sDrivetrainSubsystem.getHeading().getRadians()) < Math.toRadians(3.0);
    SmartDashboard.putBoolean("decide to feed", shootOk);
    return mReadyToFeed.update(Timer.getFPGATimestamp(), shootOk);
  }

  private Translation2d calculateTarget3D(Translation2d driveVector) {
    // find goal by alliance
    Translation2d goal;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      goal = VisionShootConstants.kBlueSpeaker;
    } else { // red
      goal = VisionShootConstants.kRedSpeaker;
    }

    // find robotToGoal
    Translation2d robotToGoal = goal.minus(sDrivetrainSubsystem.getPose().getTranslation());
    
    double shooter_rps = VisionShootConstants.kSpeakerRPSMap.get(robotToGoal.getNorm());
    double shooter_pitch_degree = VisionShootConstants.kSpeakerAngleMap.get(robotToGoal.getNorm());
    final double kMagicScalar = 0.9;
    double noteFlySpeed = shooter_rps * 5.0 / 3.0 * kMagicScalar * Math.PI * 0.021 * Math.cos(Math.toRadians(shooter_pitch_degree));
    double timeOfFly = robotToGoal.getNorm() / noteFlySpeed;
    Translation2d offsetDueToMove = driveVector.times(timeOfFly); // delta x = v * t
    Translation2d aimTargetToRobot = robotToGoal.minus(offsetDueToMove);
    SmartDashboard.putNumber("dist to goal", aimTargetToRobot.getNorm());
    return aimTargetToRobot;
  }

  private void handleFeeding() {
    sTransfer.setVoltage(Transfer.FEED_VOLTS);
    sDrivetrainSubsystem.drive(mDriveVectorSupplier.get(), 0.0, true);
    if (stateTimer.hasElapsed(0.1)) {
      // we continue shoot so we go back aiming
      mState = State.AIMING;
    }
  }

  @Override
  public void end(boolean interrupted) {
    sDrivetrainSubsystem.stop();
    sArm.stop();
    sShooter.stop();
    sTransfer.stop();
    stateTimer.stop();
    sIntake.stop();
  }

  @Override
  public boolean isFinished() {
    // we continue shoot so we never finish
    return mState == State.END;
  }
}
