package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

@Deprecated
public class VisionShootCommand extends ParallelCommandGroup {
  private final Shooter mShooter;
  private final Arm mArm;
  private final Transfer mTransfer;
  private final DrivetrainSubsystem mDrivetrain;
  private final Intake mIntake;

  private DualEdgeDelayedBoolean goShoot =
      new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 0.1, EdgeType.RISING);

  private Optional<ShootingParameters> shootingParameters_ = Optional.empty();
  private Optional<Translation2d> goalToRobot_ = Optional.empty();
  private Optional<Rotation2d> rotationTarget_ = Optional.empty();

  private DriveWithFollowHeadingCommand driveCommand;

  private StructPublisher<Translation2d> aimingTargetPublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("Goal", Translation2d.struct)
          .publish();

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
        new DriveWithFollowHeadingCommand(
            drivetrain,
            driveVectorSupplier,
            () -> rotationTarget_,
            () -> false); // drivetrain always aim towards speaker, always field relative

    addCommands(
        // drivetrain
        driveCommand,
        new InstantCommand(
            () ->
                goShoot =
                    new DualEdgeDelayedBoolean(Timer.getFPGATimestamp(), 0.1, EdgeType.RISING)),

        // shooter and arm
        Commands.run( // repeatedly change shooter and arm targets
                () -> {

                  // calculate target velocity and angle
                  calculateVisionAimingParameters(drivetrain);

                  Optional<ShootingParameters> sp = shootingParameters_;
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
                }),
        // feeder
        new WaitCommand(0.1)
            .andThen(
                Commands.run(
                        () -> {
                          if (readyToShoot()) {
                            mTransfer.setVoltage(Transfer.FEED_VOLTS);
                          } else {
                            mTransfer.stop();
                          }
                        },
                        mTransfer)),

        // intake
        Commands.run(
                () -> {
                  if (!mTransfer.isOmronDetected()) {
                    mIntake.setIntake();
                  } else {
                    mIntake.stop();
                  }
                }),

        // log
        Commands.run(
                () -> {
                  var goalToRobot = goalToRobot_;
                  if (goalToRobot.isPresent()) {
                    SmartDashboard.putNumber("distance to goal", goalToRobot.get().getNorm());
                  }
                  SmartDashboard.putBoolean("ready to shoot", readyToShoot());
                }));
    
    addRequirements(drivetrain, shooter, arm, transfer, intake);
  }

  // no delayed boolean for filter yet, need to verify whether needed
  private boolean readyToShoot() {
    return goShoot.update(
        Timer.getFPGATimestamp(),
        Math.abs(mShooter.getFollowerVelocity() - mShooter.getTargetVelocity()) < 2.0
            && Math.abs(mShooter.getMainMotorVelocity() - mShooter.getTargetVelocity()) < 2.0
            && Math.abs(mArm.getAngleDeg() - mArm.getTargetAngleDeg()) < 2.0
            && driveCommand.headingAligned());
  }

  /**
   * Calculates the Goal Position relative to robot, in field's coordinate system. If drivetrain is
   * moving, then we need to offset the goal position by a position vector which is time of note fly
   * times the robot's velocity
   *
   * @return goal position relative to robot, in field's coordinate system, unit is meter.
   */
  private Optional<Translation2d> getGoalToRobot(
      DrivetrainSubsystem drivetrainSubsystem, double shooter_pitch_degree, double shooter_rps) {
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
    double timeOfFly = getTimeOfFly(goalToRobot, shooter_pitch_degree, shooter_rps);
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
  private double getTimeOfFly(
      Translation2d goalToRobot, double shooter_pitch_degree, double shooter_rps) {
    // 100 rotations/s * 0.021PI m/rotation * 45degree shoot angle

    double noteFlySpeed =
        shooter_rps
            * 5.0
            / 3.0
            * 0.8
            * Math.PI
            * 0.021
            * Math.cos(Math.toRadians(shooter_pitch_degree));
    return goalToRobot.getNorm()
        / noteFlySpeed; // TODO: assumed note fly speed projection on the xy-plane is constant,
    // verify accuracy
  }

  /**
   * Calculate shooting parameters and rotation target from robot pose. Use iterative approach to
   * get the time of fly of the game piece. Do not return value but store result in member
   * variables.
   *
   * @param drivetrainSubsystem
   */
  private void calculateVisionAimingParameters(DrivetrainSubsystem drivetrainSubsystem) {
    double shooter_pitch_degree = 45;
    double shooter_rps = 53;
    Optional<Translation2d> finalGoalToRobot = Optional.empty();
    // iterative approach to get the goal to robot vector
    for (int i = 0; i < 5; i++) {
      var goalToRobot = getGoalToRobot(drivetrainSubsystem, shooter_pitch_degree, shooter_rps);
      if (goalToRobot.isEmpty()) {
        return;
      }
      shooter_pitch_degree = VisionShootConstants.kSpeakerAngleMap.get(goalToRobot.get().getNorm());
      shooter_rps = VisionShootConstants.kSpeakerRPSMap.get(goalToRobot.get().getNorm());
      finalGoalToRobot = goalToRobot;
    }

    shootingParameters_ = Optional.of(new ShootingParameters(shooter_rps, shooter_pitch_degree));
    rotationTarget_ =
        Optional.of(finalGoalToRobot.get().getAngle().rotateBy(Rotation2d.fromDegrees(180)));

    return;
  }
}
