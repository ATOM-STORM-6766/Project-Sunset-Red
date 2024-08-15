package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionShootConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.*;
import java.util.function.Supplier;

public class VisionShootWhileMovingCommand extends Command {

  private DrivetrainSubsystem sDrivetrainSubsystem;
  private Arm sArm;
  private Transfer sTransfer;
  private Shooter sShooter;
  private Supplier<Translation2d> mDriveVectorSupplier;

  private final PIDController mHeadingPID = new PIDController(5.0, 0, 0.5);

  private enum State {
    AIMING, FEEDING, END
  }
  private State mState = State.AIMING;

  double shooterSpeed, armAngle;

  private static final double READY_TO_FEED_DELAY = 0.1;
  private DualEdgeDelayedBoolean mReadyToFeed;

  private Timer stateTimer = new Timer();

  public VisionShootWhileMovingCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Transfer transfer,
      Shooter shooter,
      Supplier<Translation2d> driveVectorSupplier) {
    sDrivetrainSubsystem = drivetrainSubsystem;
    sArm = arm;
    sTransfer = transfer;
    sShooter = shooter;
    mDriveVectorSupplier = driveVectorSupplier;

    addRequirements(drivetrainSubsystem, arm, transfer, shooter);
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
    Translation2d targetVector = calculateTarget(driveVector);
    Rotation2d targetRobotHeading = targetVector.getAngle().rotateBy(Rotation2d.fromDegrees(180.0));
    double targetDist = targetVector.getNorm();

    // find shoot parameter
    shooterSpeed = VisionShootConstants.kSpeakerRPSMap.get(targetDist);
    armAngle = VisionShootConstants.kSpeakerAngleMap.get(targetDist);

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

  private boolean decideToFeed() {
    // find if superstruct ok
    boolean shootOk = Math.abs(sShooter.getAverageVelocity() - shooterSpeed) < 4.0
      && Math.abs(sArm.getAngleDeg() - armAngle) < 2.0
      && Math.abs(mHeadingPID.getSetpoint() - sDrivetrainSubsystem.getHeading().getRadians()) < Math.toRadians(3.0);
    
    return mReadyToFeed.update(Timer.getFPGATimestamp(), shootOk);
  }

  private Translation2d calculateTarget(Translation2d driveVector) {
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
    
    double timeOfFly = robotToGoal.getNorm() / kNoteFlySpeed;
    Translation2d offsetDueToMove = driveVector.times(timeOfFly); // delta x = v * t
    // TODO : IS IT MINUS() OR PLUS() BELOW ?
    Translation2d aimTargetToRobot = robotToGoal.minus(offsetDueToMove);
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
  }

  @Override
  public boolean isFinished() {
    // we continue shoot so we never finish
    return false;
  }
}
