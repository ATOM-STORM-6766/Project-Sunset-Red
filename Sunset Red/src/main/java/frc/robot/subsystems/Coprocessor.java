package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Coprocessor extends SubsystemBase {
  private static Coprocessor mCoprocessor = new Coprocessor();
  private static final double ACCEPTABLE_AMBIGUITY_THRESHOLD = 0.2;

  public static Coprocessor getInstance() {
    return mCoprocessor;
  }

  private Coprocessor() {
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  private PhotonCamera ov9281 = new PhotonCamera("OV9281");
  private Transform3d kRobotToCamera = new Transform3d(0.25, 0.06, 0.25,
      new Rotation3d(0, 220.0 / 180 * Math.PI, Math.PI));
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

    Optional<EstimatedRobotPose> newEstimatedRobotPose = Optional.empty();

    /*
      * Filter out tags that are too close to the edge of the camera frame or have
      * high ambiguity or are too far away

     */
    var acceptableTargets = result.getTargets().stream()
        .filter(target -> target.getArea() > 0.11 && target.getYaw() < 30
            && target.getYaw() > -30
            && target.getPitch() < 20
            && target.getPitch() > -20
            && target.getPoseAmbiguity() < ACCEPTABLE_AMBIGUITY_THRESHOLD)
        .collect(Collectors.toList());

    // Logging original tags
    String[] originalTagsInfo = new String[result.getTargets().size()];
    for (int i = 0; i < result.getTargets().size(); i++) {
      var target = result.getTargets().get(i);
      originalTagsInfo[i] = String.format("ID: %d, Ambiguity: %.3f", target.getFiducialId(), target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Original Tags", originalTagsInfo);

    // Logging accepted tags
    String[] acceptedTagsInfo = new String[acceptableTargets.size()];
    for (int i = 0; i < acceptableTargets.size(); i++) {
      var target = acceptableTargets.get(i);
      acceptedTagsInfo[i] = String.format("ID: %d, Ambiguity: %.3f", target.getFiducialId(), target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Accepted Tags", acceptedTagsInfo);

    if (!acceptableTargets.isEmpty()) {
      result.targets.clear();
      result.targets.addAll(acceptableTargets);
      newEstimatedRobotPose = photonPoseEstimator.update(result);
      SmartDashboard.putString("Vision Estimation Mode", newEstimatedRobotPose.get().strategy.toString());

      // Logging updated results
      if (newEstimatedRobotPose.isPresent()) {
        var pose = newEstimatedRobotPose.get().estimatedPose;
        String[] poseInfo = {
          String.format("X: %.2f", pose.getX()),
          String.format("Y: %.2f", pose.getY()),
          String.format("Z: %.2f", pose.getZ()),
          String.format("Yaw: %.2f", pose.getRotation().getZ()),
          String.format("Timestamp: %.3f", newEstimatedRobotPose.get().timestampSeconds)
        };
        SmartDashboard.putStringArray("Estimated Pose", poseInfo);
      } else {
        SmartDashboard.putStringArray("Estimated Pose", new String[] { "Failed to estimate pose" });
      }
    } else {
      SmartDashboard.putString("Vision Estimation Mode", "No Acceptable Targets");
      SmartDashboard.putStringArray("Estimated Pose", new String[] { "No pose estimated" });
      return Optional.empty();
    }

    newEstimatedRobotPose.ifPresent(pose -> currRPpublisher.set(pose.estimatedPose.toPose2d()));

    // Additional logging
    SmartDashboard.putNumber("Number of Targets", result.getTargets().size());

    return newEstimatedRobotPose;
  }

  @Override
  public void periodic() {
  }
}