package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
  private Pose2d lastRobotPose = new Pose2d(); // get Pose from odom
  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, ov9281, kRobotToCamera);

  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("RobotPose", Pose2d.struct)
          .publish();

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  double lastTime = Timer.getFPGATimestamp();

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> currRobotPose = getEstimatedGlobalPose(lastRobotPose);
    if (currRobotPose.isPresent()) {
      // publisher.set(currRobotPose.get().estimatedPose.toPose2d());
      lastRobotPose = currRobotPose.get().estimatedPose.toPose2d();
      SmartDashboard.putBoolean("photon_pose_present", true);
    } else {
      // publisher.set(lastRobotPose);
      SmartDashboard.putBoolean("photon_pose_present", false);
    }
  }
}
