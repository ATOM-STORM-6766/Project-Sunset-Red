package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.Optional;

public class DriveToAmpCommand extends Command {
  private final DrivetrainSubsystem sDrivetrainSubsystem;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private Pose2d mTargetPose;
  private static final double DISTANCE_OFFSET = 0.50; // tag wall to robot center in meters.

  private static final int RED_TAG_ID = 5; // Amp tag ID
  private static final int BLUE_TAG_ID = 6; // Amp tag ID

  public DriveToAmpCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.sDrivetrainSubsystem = drivetrainSubsystem;
    TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    xController = new ProfiledPIDController(4.0, 0, 0, constraints);
    xController.setTolerance(0.05);
    yController = new ProfiledPIDController(4.0, 0, 0, constraints);
    yController.setTolerance(0.05);
    rotationController =
        new ProfiledPIDController(
            4.0,
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

    mTargetPose = findAmpPose(currentPose);
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

  private Pose2d findAmpPose(Pose2d currentPose) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int ampTagId = alliance == Alliance.Blue ? BLUE_TAG_ID : RED_TAG_ID;

    Optional<Pose3d> ampTagPoseOptional =
        ApriltagCoprocessor.getInstance().getAprilTagPose(ampTagId);

    if (ampTagPoseOptional.isPresent()) {
      Pose2d ampPose = ampTagPoseOptional.get().toPose2d();
      return ampPose.transformBy(
          new Transform2d(new Translation2d(DISTANCE_OFFSET, 0), Rotation2d.fromDegrees(180)));
    } else {
      // Handle the case where the tag pose is not found, then return the current pose.
      return currentPose;
    }
  }
}
