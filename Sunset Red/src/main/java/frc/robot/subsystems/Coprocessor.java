package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Coprocessor extends SubsystemBase {
  private static Coprocessor mCoprocessor = new Coprocessor();

  public static Coprocessor getInstance() {
    return mCoprocessor;
  }

  private Coprocessor() {}

  private PhotonCamera ov9281 = new PhotonCamera("OV9281");
  private Transform3d kRobotToCamera =
      new Transform3d(0.25, 0.06, 0.25, new Rotation3d(0, 210.0 / 180 * Math.PI, Math.PI));
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  // private EstimatedRobotPose lastRobotPose = new EstimatedRobotPose(new Pose3d(), -1, null,
  // null); // get Pose from odom
  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, ov9281, kRobotToCamera);

  private double lastEstimateTimestamp = 0.0;
  private Optional<EstimatedRobotPose> lastVisionEstimatedPose = Optional.empty();

  StructPublisher<Pose2d> lastRPpublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("lastRobotPose", Pose2d.struct)
          .publish();
  StructPublisher<Pose2d> currRPpublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("currRobotPose", Pose2d.struct)
          .publish();

  /**
   * Should run this function as frequent as possible
   *
   * @param prevEstimatedRobotPose latest estimated robot pose (ideally from odom)
   * @return vision estimated robot pose using Optional class to signal vision presence
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var newEstimatedRobotPose = photonPoseEstimator.update();
    if (newEstimatedRobotPose.isEmpty()) {
      // System.out.println("no new vision position estinate");
      return Optional.empty();
    } else {
      currRPpublisher.set(newEstimatedRobotPose.get().estimatedPose.toPose2d());
    }
    // filter out if estimated translation change a lot since last vision update (if it's recent)
    double visionDeltaT = newEstimatedRobotPose.get().timestampSeconds - lastEstimateTimestamp;
    lastEstimateTimestamp = newEstimatedRobotPose.get().timestampSeconds;
    if (visionDeltaT < 0.1
        && lastVisionEstimatedPose.isPresent()
        && lastVisionEstimatedPose
                .get()
                .estimatedPose
                .minus(newEstimatedRobotPose.get().estimatedPose)
                .getTranslation()
                .getNorm()
            > 0.2) {
      System.out.println("moving too quick");
      return Optional.empty();
    }

    // filter apriltag if close to camera edge
    PhotonTrackedTarget target = ov9281.getLatestResult().getBestTarget();
    if (target.getYaw() > 30
        || target.getYaw() < -30
        || target.getPitch() > 20
        || target.getPitch() < -20) { // assume signal target ?
      System.out.println("target on camera edge");
      return Optional.empty();
    }
    lastVisionEstimatedPose = newEstimatedRobotPose;
    return lastVisionEstimatedPose;
  }

  @Override
  public void periodic() {}
}
