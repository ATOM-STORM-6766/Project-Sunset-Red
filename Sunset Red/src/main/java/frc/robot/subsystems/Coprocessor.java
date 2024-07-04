package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib6907.DelayedBoolean;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Coprocessor extends SubsystemBase {
  private static Coprocessor mCoprocessor = new Coprocessor();
  private static final double ACCEPTABLE_AMBIGUITY_THRESHOLD = 0.15;
  private DelayedBoolean multiTagDelayedBoolean;
  private static final double MULTI_TAG_DELAY = 0.5;

  public static Coprocessor getInstance() {
    return mCoprocessor;
  }

  private Coprocessor() {
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
    double currentTime = Timer.getFPGATimestamp();
    multiTagDelayedBoolean = new DelayedBoolean(currentTime, MULTI_TAG_DELAY);
  }

  private PhotonCamera ov9281 = new PhotonCamera("OV9281");
  // original 0.28
  private Transform3d kRobotToCamera = new Transform3d(-0.28, -0.06, 0.25,
      new Rotation3d(0, 220.0 / 180 * Math.PI, 0));
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ov9281, kRobotToCamera);

  StructPublisher<Pose2d> lastRPpublisher = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard")
      .getStructTopic("lastRobotPose", Pose2d.struct)
      .publish();
  StructPublisher<Pose2d> currRPpublisher = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard")
      .getStructTopic("currRobotPose", Pose2d.struct)
      .publish();

  private Translation2d lastVisionEstimatedPose = null;
  private double lastVisionEstimatedPoseTimestamp = 0;

  /**
   * Should run this function as frequent as possible
   *
   * @param prevEstimatedRobotPose latest estimated robot pose (ideally from odom)
   * @return vision estimated robot pose using Optional class to signal vision
   *         presence
   */
  public Optional<EstimatedRobotPose> updateEstimatedGlobalPose(
      Pose2d prevEstimatedRobotPose, Translation2d chassisVelMS) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    var result = ov9281.getLatestResult();
    if (!result.hasTargets()) {
      SmartDashboard.putString("Vision Estimation Mode", "No Detected Targets");
      return Optional.empty();
    }

    // Before filtering, publish the result to Smartdashboard
    SmartDashboard.putNumber("Number of Targets", result.getTargets().size());
    SmartDashboard.putNumber("Timestamp", result.getTimestampSeconds());

    logOriginalTags(result);

    var acceptableTargets = filterAcceptableTargets(result);
    SmartDashboard.putNumber("Number of Accepted Targets", acceptableTargets.size());

    logAcceptedTags(acceptableTargets);

    if (acceptableTargets.isEmpty()) {
      SmartDashboard.putString("Vision Estimation Mode", "No Acceptable Targets");
      return Optional.empty();
    }

    PoseStrategy newStrategy = determineStrategy(acceptableTargets);
    photonPoseEstimator.setPrimaryStrategy(newStrategy);

    result.targets.clear();
    result.targets.addAll(acceptableTargets);

    Optional<EstimatedRobotPose> newEstimatedRobotPose = photonPoseEstimator.update(result);

    if (newEstimatedRobotPose.isPresent()) {
      SmartDashboard.putString("Vision Estimation Mode", newEstimatedRobotPose.get().strategy.toString());
      logEstimatedPose(newEstimatedRobotPose.get());
      currRPpublisher.set(newEstimatedRobotPose.get().estimatedPose.toPose2d());
    } else {
      SmartDashboard.putStringArray("Estimated Pose", new String[] { "Failed to estimate pose" });
    }

    var return_pose = newEstimatedRobotPose;

    // // velocity filter
    if (lastVisionEstimatedPose != null && newEstimatedRobotPose.isPresent()
        && lastVisionEstimatedPoseTimestamp != newEstimatedRobotPose.get().timestampSeconds) {
      Translation2d visionVelMS = newEstimatedRobotPose.get().estimatedPose.getTranslation().toTranslation2d()
          .minus(lastVisionEstimatedPose)
          .div(newEstimatedRobotPose.get().timestampSeconds - lastVisionEstimatedPoseTimestamp);
      SmartDashboard.putString("vision vel ms", visionVelMS.toString());
      SmartDashboard.putNumber("velocity delta v", visionVelMS.minus(chassisVelMS).getNorm());
      if (visionVelMS.minus(chassisVelMS).getNorm() > 0.5) {
        return_pose = Optional.empty();
      }
    }

    if (newEstimatedRobotPose.isPresent()) {
      lastVisionEstimatedPose = newEstimatedRobotPose.get().estimatedPose.getTranslation().toTranslation2d();
      lastVisionEstimatedPoseTimestamp = newEstimatedRobotPose.get().timestampSeconds;
    }

    return return_pose;
  }

  private PoseStrategy determineStrategy(List<PhotonTrackedTarget> acceptableTargets) {
    double currentTime = Timer.getFPGATimestamp();
    boolean isMultiTag = acceptableTargets.size() > 1;

    boolean useMultiTag = multiTagDelayedBoolean.update(currentTime, isMultiTag);

    if (useMultiTag) {
       return PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
      //return PoseStrategy.AVERAGE_BEST_TARGETS;
    } else {
      return PoseStrategy.AVERAGE_BEST_TARGETS;
    }
  }

  private void logOriginalTags(PhotonPipelineResult result) {
    String[] originalTagsInfo = new String[result.getTargets().size()];
    for (int i = 0; i < result.getTargets().size(); i++) {
      var target = result.getTargets().get(i);
      originalTagsInfo[i] = String.format(
          "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
          target.getFiducialId(), target.getArea(), target.getYaw(), target.getPitch(), target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Original Tags", originalTagsInfo);
  }

  private List<PhotonTrackedTarget> filterAcceptableTargets(PhotonPipelineResult result) {
    return result.getTargets().stream()
        .filter(target -> {
          boolean isAcceptable = target.getArea() > 0.11
              && target.getPoseAmbiguity() < ACCEPTABLE_AMBIGUITY_THRESHOLD
              && Math.abs(target.getYaw()) < 30
              && Math.abs(target.getPitch()) < 25;
/*
 * && Math.abs(target.getYaw()) < 30
              && Math.abs(target.getPitch()) < 20
 */
          SmartDashboard.putString("Filtering Log", String.format(
              "ID: %d, Acceptable: %b, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
              target.getFiducialId(), isAcceptable, target.getArea(), target.getYaw(), target.getPitch(),
              target.getPoseAmbiguity()));
          return isAcceptable;
        })
        .collect(Collectors.toList());
  }

  private void logAcceptedTags(List<PhotonTrackedTarget> acceptableTargets) {
    String[] acceptedTagsInfo = new String[acceptableTargets.size()];
    for (int i = 0; i < acceptableTargets.size(); i++) {
      var target = acceptableTargets.get(i);
      acceptedTagsInfo[i] = String.format(
          "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
          target.getFiducialId(), target.getArea(), target.getYaw(), target.getPitch(), target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Accepted Tags", acceptedTagsInfo);
  }

  private void logEstimatedPose(EstimatedRobotPose estimatedPose) {
    var pose = estimatedPose.estimatedPose;
    String[] poseInfo = {
        String.format("X: %.2f", pose.getX()),
        String.format("Y: %.2f", pose.getY()),
        String.format("Z: %.2f", pose.getZ()),
        String.format("Yaw: %.2f", pose.getRotation().getZ()),
        String.format("Timestamp: %.3f", estimatedPose.timestampSeconds)
    };
    SmartDashboard.putStringArray("Estimated Pose", poseInfo);
  }

  @Override
  public void periodic() {
    // Add any periodic tasks if needed
  }
}
