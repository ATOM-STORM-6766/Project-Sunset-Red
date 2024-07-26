package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ApriltagCoprocessor;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.List;

public class DriveToNearestTrapCommand extends Command {
  private final DrivetrainSubsystem sDrivetrainSubsystem;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private Pose2d mTargetPose;
  private double DISTANCE_OFFSET = 0.78; // tag wall to robot center in meters.

  private static final int[] RED_TAG_IDS = {11, 12, 13}; // Trap tag IDs
  private static final int[] BLUE_TAG_IDS = {14, 15, 16}; // Trap tag IDs

  public DriveToNearestTrapCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.sDrivetrainSubsystem = drivetrainSubsystem;

    TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    xController = new ProfiledPIDController(5.0, 0, 0, constraints);
    xController.setTolerance(0.05);
    yController = new ProfiledPIDController(5.0, 0, 0, constraints);
    yController.setTolerance(0.05);
    rotationController =
        new ProfiledPIDController(
            5.0,
            0,
            0,
            new TrapezoidProfile.Constraints(Math.toRadians(540.0), Math.toRadians(720.0)));

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Math.toRadians(2.0));

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = sDrivetrainSubsystem.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    rotationController.reset(currentPose.getRotation().getRadians());

    mTargetPose = findNearestTrapPose(currentPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = sDrivetrainSubsystem.getPose();

    double xSpeed =
        xController.calculate(currentPose.getX(), mTargetPose.getX())
            + xController.getSetpoint().velocity;
    double ySpeed =
        yController.calculate(currentPose.getY(), mTargetPose.getY())
            + yController.getSetpoint().velocity;
    double rotationSpeed =
        rotationController.calculate(
                currentPose.getRotation().getRadians(), mTargetPose.getRotation().getRadians())
            + rotationController.getSetpoint().velocity;

    sDrivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    sDrivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && rotationController.atGoal();
  }

  private Pose2d findNearestTrapPose(Pose2d currentPose) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int[] trapTagIds = (alliance == Alliance.Blue) ? BLUE_TAG_IDS : RED_TAG_IDS;

    List<Pose2d> trapPoses = new ArrayList<>();
    for (int tagId : trapTagIds) {
      ApriltagCoprocessor.getInstance()
          .getAprilTagPose(tagId)
          .ifPresent(tagPose -> trapPoses.add(tagPose.toPose2d()));
    }

    Pose2d nearestTrap = trapPoses.get(0);
    double minDistance = Double.MAX_VALUE;

    for (Pose2d trap : trapPoses) {
      double distance = currentPose.getTranslation().getDistance(trap.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        nearestTrap = trap;
      }
    }

    // Adjust the final pose to be in front of the trap
    return nearestTrap.transformBy(
        new Transform2d(new Translation2d(DISTANCE_OFFSET, 0), new Rotation2d()));
  }
}
