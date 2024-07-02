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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

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

  private Coprocessor() {
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  private PhotonCamera ov9281 = new PhotonCamera("OV9281");
  private Transform3d kRobotToCamera =
      new Transform3d(0.25, 0.06, 0.25, new Rotation3d(0, 220.0 / 180 * Math.PI, Math.PI));
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  // private EstimatedRobotPose lastRobotPose = new EstimatedRobotPose(new Pose3d(), -1, null,
  // null); // get Pose from odom
  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ov9281, kRobotToCamera);

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
  public Optional<EstimatedRobotPose> updateEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, Translation2d chassisVelMS) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var newEstimatedRobotPose = photonPoseEstimator.update();
    
    if (newEstimatedRobotPose.isEmpty()) {
      return Optional.empty();
    } else {
      currRPpublisher.set(newEstimatedRobotPose.get().estimatedPose.toPose2d());
    }

    boolean all_trustworthy = true;
    double[] target_ambiguities = newEstimatedRobotPose.get().targetsUsed.stream().mapToDouble(x -> x.getPoseAmbiguity()).toArray();
    for(var target : newEstimatedRobotPose.get().targetsUsed){
      if(target.getPoseAmbiguity() != -1 || target.getPoseAmbiguity() > 0.2){
        all_trustworthy = false;
      }
    }

    SmartDashboard.putNumberArray("ambiguities", target_ambiguities);



    if(all_trustworthy){
      return newEstimatedRobotPose;
    }
    return Optional.empty();
    
    /*
     * Old processing logic, subed by using multi tag strategy and ambiguouty check 
     */
    // double visionDeltaT = newEstimatedRobotPose.get().timestampSeconds - lastEstimateTimestamp;
    // if(lastVisionEstimatedPose.isPresent()) {
    //   Translation2d visionVelMS = newEstimatedRobotPose.get().estimatedPose.getTranslation().minus(lastVisionEstimatedPose.get().estimatedPose.getTranslation()).toTranslation2d().div(visionDeltaT);
    //   if (visionVelMS.minus(chassisVelMS).getNorm() > 0.5) { // TODO: test
    //     return Optional.empty();
    //   }
    // if (visionDeltaT < 0.1
    //     && lastVisionEstimatedPose.isPresent()
    //     && lastVisionEstimatedPose
    //             .get()
    //             .estimatedPose
    //             .minus(newEstimatedRobotPose.get().estimatedPose)
    //             .getTranslation()
    //             .getNorm()
    //         > 0.2) {
    // return Optional.empty();
    // }

    // filter apriltag if close to camera edge
    // PhotonTrackedTarget target = ov9281.getLatestResult().getBestTarget();
    // // PhotonTrackedTarget target = ov9281.getLatestResult()
    // if (target == null || target.
    //     || target.getYaw() > 30
    //     || target.getYaw() < -30
    //     || target.getPitch() > 20
    //     || target.getPitch() < -20
    //     || target.getArea() < 0.11) { // assume signal target ?
    //   System.out.println("target on camera edge");
    //   return Optional.empty();
    // }

    
    

    // lastVisionEstimatedPose = newEstimatedRobotPose;
    // lastEstimateTimestamp = newEstimatedRobotPose.get().timestampSeconds;
    // return lastVisionEstimatedPose;
  }

  @Override
  public void periodic() {}
}
