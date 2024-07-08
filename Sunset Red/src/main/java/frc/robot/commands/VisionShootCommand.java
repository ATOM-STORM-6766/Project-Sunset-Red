package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionShootConstants;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionShootCommand extends ParallelCommandGroup {
  private final Shooter mShooter;
  private final Arm mArm;
  private final Transfer mTransfer;
  private final DrivetrainSubsystem mDrivetrain;
  private final Intake mIntake;

  private DualEdgeDelayedBoolean goShoot =
      new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 0.1, EdgeType.RISING);

  private SnapToAngleCommand driveCommand;

  private StructPublisher<Translation2d> aimingTargetPublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("Goal", Translation2d.struct)
          .publish();

  // 100 rotations/s * 0.021PI m/rotation * 45degree shoot angle
  public static final double kNoteFlySpeed = 80.0 * Math.PI * 0.021 * Math.cos(Math.PI / 4);

  public VisionShootCommand(
      Shooter shooter,
      Arm arm,
      Transfer transfer,
      DrivetrainSubsystem drivetrain,
      Intake intake,
      Supplier<Translation2d> driveVectorSupplier) {
    mShooter = shooter;
    mArm = arm;
    mTransfer = transfer;
    mDrivetrain = drivetrain;
    mIntake = intake;
    driveCommand =
        new SnapToAngleCommand(
            drivetrain,
            driveVectorSupplier,
            () -> getRotationTarget(mDrivetrain),
            () -> false); // drivetrain always aim towards speaker, always field relative

    addCommands(

        // drivetrain
        driveCommand,

        // shooter and arm
        new RepeatCommand(
            new InstantCommand( // repeatedly change shooter and arm targets
                () -> {
                  Optional<ShootingParameters> sp = getShootingParameters(mDrivetrain);
                  if (sp.isPresent()) {
                    mShooter.setTargetVelocity(sp.get().speed_rps);
                    mArm.setAngle(sp.get().angle_deg);
                  } else { // usually should not enter here
                    mShooter.stop();
                    // soft home
                    if (mArm.getAngleDeg() < ArmConstants.ARM_REST_ANGLE + 3.0) {
                      mArm.stop();
                    } else {
                      mArm.setAngle(ArmConstants.ARM_REST_ANGLE);
                    }
                  }
                })),

        // feeder
        new WaitCommand(1.0)
            .andThen(
                new RepeatCommand(
                    new InstantCommand(
                        () -> {
                          if (readyToShoot()) {
                            mTransfer.setVoltage(Transfer.FEED_VOLTS);
                          } else {
                            mTransfer.stop();
                          }
                        },
                        mTransfer))),

        // intake
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  if (!mTransfer.isOmronDetected()) {
                    mIntake.setIntake();
                  } else {
                    mIntake.stop();
                  }
                })),

        // log
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  var goalToRobot = getGoalToRobot(mDrivetrain);
                  if (goalToRobot.isPresent()) {
                    SmartDashboard.putNumber("distance to goal", goalToRobot.get().getNorm());
                  }
                  SmartDashboard.putBoolean("ready to shoot", readyToShoot());
                })));
  }

  // no delayed boolean for filter yet, need to verify whether needed
  private boolean readyToShoot() {
    return goShoot.update(
        Timer
            .getFPGATimestamp(), /* Math.abs(mShooter.getFollowerVelocity() - mShooter.getTargetVelocity()) < 2.0
                                 && Math.abs(mShooter.getMainMotorVelocity() - mShooter.getTargetVelocity()) < 2.0
                                 && Math.abs(mArm.getAngleDeg() - mArm.getTargetAngleDeg()) < 1.0
                                 && */
        driveCommand.isAligned());
  }

  /**
   * Calculates the Goal Position relative to robot, in field's coordinate system. If drivetrain is
   * moving, then we need to offset the goal position by a position vector which is time of note fly
   * times the robot's velocity
   *
   * @return goal position relative to robot, in field's coordinate system, unit is meter.
   */
  private Optional<Translation2d> getGoalToRobot(DrivetrainSubsystem drivetrainSubsystem) {
    Translation2d robotToField = drivetrainSubsystem.getPose().getTranslation();
    Optional<Alliance> a = DriverStation.getAlliance();
    if (a.isEmpty()) {
      return Optional.empty();
    }

    Translation2d goalToField =
        a.get() == Alliance.Red
            ? VisionShootConstants.kRedSpeaker
            : VisionShootConstants.kBlueSpeaker;

    Translation2d goalToRobot = goalToField.minus(robotToField);
    double timeOfFly = getTimeOfFly(goalToRobot);
    Translation2d offsetDueToMove = mDrivetrain.getVelocity().times(timeOfFly); // delta x = v * t
    Translation2d aimTargetToRobot =
        goalToRobot.plus(offsetDueToMove); // goal position plus offset due to robot motion
    aimingTargetPublisher.set(aimTargetToRobot.plus(robotToField));
    return Optional.of(aimTargetToRobot);
  }

  /**
   * Estimate the time that the game piece flies. Use to compensate robot velocity if shoot while
   * moving.
   *
   * @param goalToRobot
   * @return
   */
  private double getTimeOfFly(Translation2d goalToRobot) {
    return goalToRobot.getNorm()
        / kNoteFlySpeed; // TODO: assumed note fly speed projection on the xy-plane is constant,
    // verify accuracy
  }

  /**
   * Return the angle that makes the robot points to the goal
   *
   * @return the angle that the robot should rotate to in order to aim to goal.
   */
  private Optional<Rotation2d> getRotationTarget(DrivetrainSubsystem drivetrainSubsystem) {
    var goalToRobot = getGoalToRobot(drivetrainSubsystem);
    if (goalToRobot.isEmpty()) {
      return Optional.empty();
    }
    // rotate by 180 degrees because shooter is on the back side of the robot, intake is front
    return Optional.of(goalToRobot.get().getAngle().rotateBy(Rotation2d.fromDegrees(180)));
  }

  /**
   * calculate shooting parameters from robot pose.
   *
   * @param drivetrainSubsystem
   * @return shooting parameter that is used to set shooter and arm target.
   */
  private Optional<ShootingParameters> getShootingParameters(
      DrivetrainSubsystem drivetrainSubsystem) {
    var goalToRobot = getGoalToRobot(drivetrainSubsystem);
    if (goalToRobot.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        new ShootingParameters(
            100, VisionShootConstants.kSpeakerAngleMap.get(goalToRobot.get().getNorm())));
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
