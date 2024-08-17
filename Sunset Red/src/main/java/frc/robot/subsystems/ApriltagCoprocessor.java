package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.Comparator;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ApriltagCoprocessor extends SubsystemBase {
  private static ApriltagCoprocessor mCoprocessor;
  private static final double ACCEPTABLE_AMBIGUITY_THRESHOLD = 0.2;
  private static final double ACCEPTABLE_YAW_THRESHOLD = 30;
  private static final double ACCEPTABLE_PITCH_THRESHOLD = 25;
  private static final double MULTI_TAG_DELAY = 0.1;
  // if abs(Pose3d.Z) > Z_Margin, don't update
  private static final double Z_MARGIN = 0.35;

  // chassis speed filter
  private static final double STATIC_DELTAV_BOUND = 1.0; // meter per sec
  private static final double MOVING_DELTAV_BOUND = 2.0; // meter per sec

  // read-only data class from Java 14: keyword 'record'
  public record VisionObservation(EstimatedRobotPose estimate, double xyDev, double angleDev, String camname) {}

  private PhotonCamera ApriltagCamShooterSide = new PhotonCamera("TagCamShooterSide");
  private PhotonCamera ApriltagCamIntakeSide = new PhotonCamera("TagCamIntakeSide");
  private PhotonCamera ApriltagCamShooterLongFocal = new PhotonCamera("TagCamShooterSideLongFocal");

  public Transform3d kRobotToCameraForShooterSide =
      new Transform3d(-0.28, -0.105, 0.25, new Rotation3d(Units.degreesToRadians(180),
          Units.degreesToRadians(-40), Units.degreesToRadians(180)));

  private Transform3d kRobotToCameraForIntakeSide =
      new Transform3d(0.42, 0.06, 0.20, new Rotation3d(Units.degreesToRadians(180),
          Units.degreesToRadians(-45), Units.degreesToRadians(0)));

  // new camera and old camera are inverse (due to different manufacturer), so roll is 0 and 180
  // degrees
  private Transform3d kRobotToCameraForShooterLongFocal =
      new Transform3d(-0.28, -0.04, 0.25, new Rotation3d(Units.degreesToRadians(0),
          Units.degreesToRadians(-20), Units.degreesToRadians(180)));

  public final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  PhotonPoseEstimator photonPoseEstimatorForShooterSide =
      new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          ApriltagCamShooterSide, kRobotToCameraForShooterSide);
  PhotonPoseEstimator photonPoseEstimatorForIntakeSide =
      new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          ApriltagCamIntakeSide, kRobotToCameraForIntakeSide);
  PhotonPoseEstimator photonPoseEstimatorForShooterLongFocal =
      new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          ApriltagCamShooterLongFocal, kRobotToCameraForShooterLongFocal);

  StructPublisher<Pose3d> intakeSidePublisher =
      NetworkTableInstance.getDefault().getTable("SmartDashboard")
          .getStructTopic("Intake Side Estimated Pose", Pose3d.struct).publish();

  StructPublisher<Pose3d> shooterSidePublisher =
      NetworkTableInstance.getDefault().getTable("SmartDashboard")
          .getStructTopic("Shooter Side Estimated Pose", Pose3d.struct).publish();

  StructPublisher<Pose3d> shooterLongFocalPublisher =
      NetworkTableInstance.getDefault().getTable("SmartDashboard")
          .getStructTopic("Shooter Long Focal Estimated Pose", Pose3d.struct).publish();

  // list order: shooterSide, IntakeSide, ShooterLongFocal
  private ArrayList<DualEdgeDelayedBoolean> lstMultiTagDelayedBoolean;
  private ArrayList<PhotonCamera> cameras = new ArrayList<>(
      Arrays.asList(ApriltagCamShooterSide, ApriltagCamIntakeSide, ApriltagCamShooterLongFocal));
  private ArrayList<PhotonPoseEstimator> estimators =
      new ArrayList<>(Arrays.asList(photonPoseEstimatorForShooterSide,
          photonPoseEstimatorForIntakeSide, photonPoseEstimatorForShooterLongFocal));
  private ArrayList<StructPublisher<Pose3d>> publishers = new ArrayList<>(
      Arrays.asList(shooterSidePublisher, intakeSidePublisher, shooterLongFocalPublisher));
  // camera-wise area filter bounds
  private static final List<Double> kAreaBounds = List.of(0.1, 0.1, 0.2);
  // camera-wise dev factors
  // applied dev = factor * (dist^2) / tagcount
  // factor=0.02 means dev=0.08 for single tag dist=2m
  private static final double[] xyDevFactors = new double[] {0.02, 0.02, 0.05}; // 0.005, 0.005, 0.01
  private static final double[] angleDevFactors = new double[] {0.05, 0.05, 0.1};


  private ApriltagCoprocessor() {
    photonPoseEstimatorForShooterSide.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonPoseEstimatorForIntakeSide.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonPoseEstimatorForShooterLongFocal
        .setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    double currentTime = Timer.getFPGATimestamp();
    lstMultiTagDelayedBoolean = new ArrayList<>(
        Arrays.asList(new DualEdgeDelayedBoolean(currentTime, MULTI_TAG_DELAY, EdgeType.RISING),
            new DualEdgeDelayedBoolean(currentTime, MULTI_TAG_DELAY, EdgeType.RISING),
            new DualEdgeDelayedBoolean(currentTime, MULTI_TAG_DELAY, EdgeType.RISING)));
  }

  // Public method to provide access to the instance
  public static ApriltagCoprocessor getInstance() {
    if (mCoprocessor == null) {
      mCoprocessor = new ApriltagCoprocessor();
    }
    return mCoprocessor;
  }

  StructPublisher<Pose2d> lastRPpublisher = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard").getStructTopic("lastRobotPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> currRPpublisher = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard").getStructTopic("currRobotPose", Pose2d.struct).publish();

  private Translation2d[] lastVisionEstimatedPose = new Translation2d[3];
  private double[] lastVisionEstimatedPoseTimestamp = new double[] {0.0, 0.0, 0.0};

  private Optional<EstimatedRobotPose> processCameraResult(int camIdx,
      Pose2d prevEstimatedRobotPose) {
    PhotonCamera camera = cameras.get(camIdx);
    PhotonPoseEstimator photonPoseEstimator = estimators.get(camIdx);

    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      SmartDashboard.putString("Vision Estimation Mode (" + camera.getName() + ")",
          "No Detected Targets");
      return Optional.empty();
    }

    SmartDashboard.putNumber("Number of Targets (" + camera.getName() + ")",
        result.getTargets().size());
    SmartDashboard.putNumber("Timestamp (" + camera.getName() + ")", result.getTimestampSeconds());

    logOriginalTags(result, camera.getName());

    var acceptableTargets = filterAcceptableTargets(result, kAreaBounds.get(camIdx));
    SmartDashboard.putNumber("Number of Accepted Targets (" + camera.getName() + ")",
        acceptableTargets.size());

    logAcceptedTags(acceptableTargets, camera.getName());
    logTagAreas(acceptableTargets, camera.getName()); // Log tag areas

    if (acceptableTargets.isEmpty()) {
      SmartDashboard.putString("Vision Estimation Mode (" + camera.getName() + ")",
          "No Acceptable Targets");
      return Optional.empty();
    }

    PoseStrategy newStrategy = determineStrategy(camIdx, acceptableTargets);
    photonPoseEstimator.setPrimaryStrategy(newStrategy);

    result.targets.clear();
    result.targets.addAll(acceptableTargets);

    Optional<EstimatedRobotPose> newEstimatedRobotPose = photonPoseEstimator.update(result);

    if (newEstimatedRobotPose.isPresent()) {
      SmartDashboard.putString("Vision Estimation Mode (" + camera.getName() + ")",
          newEstimatedRobotPose.get().strategy.toString());
      switch (camera.getName()) {
        case "TagCamShooterSide":
          shooterSidePublisher.set(newEstimatedRobotPose.get().estimatedPose);
          break;
        case "TagCamShooterSideLongFocal":
          shooterLongFocalPublisher.set(newEstimatedRobotPose.get().estimatedPose);
          break;
        case "TagCamIntakeSide":
          intakeSidePublisher.set(newEstimatedRobotPose.get().estimatedPose);
          break;
        default:
          System.err.print("no such camera should be present: " + camera.getName());
          // throw exception?
          break;
      }
    } else {
      SmartDashboard.putStringArray("Estimated Pose (" + camera.getName() + ")",
          new String[] {"Failed to estimate pose"});
    }

    return newEstimatedRobotPose;
  }

  private void logTagAreas(List<PhotonTrackedTarget> targets, String cameraName) {
    double totalArea = targets.stream().mapToDouble(PhotonTrackedTarget::getArea).sum();
    SmartDashboard.putNumber("Total Tag Area (" + cameraName + ")", totalArea);
  }

  /***
   * update pose estimate by each camera. Strategy taken partially from 6328
   * 
   * @param prevEstimatedRobotPose
   * @param chassisVelMS
   * @return good observations from cameras, ordered from oldest to latest
   */
  public List<VisionObservation> updateEstimatedGlobalPose(Pose2d prevEstimatedRobotPose,
      Translation2d chassisVelMS) {

    List<VisionObservation> allObservation = new ArrayList<>(cameras.size());
    for (int i = 0; i < cameras.size(); i++) {
      Optional<EstimatedRobotPose> camresult = processCameraResult(i, prevEstimatedRobotPose);
      if (!camresult.isPresent()) {
        SmartDashboard.putString("Vision Update Status (" + cameras.get(i).getName() + ")",
            "NO_PROCESSED_RESULT");
        continue;
      }

      // Z filter for estimations under ground
      if (Math.abs(camresult.get().estimatedPose.getZ()) > Z_MARGIN) {
        SmartDashboard.putString("Vision Update Status (" + cameras.get(i).getName() + ")",
            "Z_POSE_INCORRECT");
        continue;
      }

      // chassis velocity filter
      if (lastVisionEstimatedPose[i] != null
          && lastVisionEstimatedPoseTimestamp[i] != camresult.get().timestampSeconds) {
        Translation2d visionVelMS = camresult.get().estimatedPose.getTranslation().toTranslation2d()
            .minus(lastVisionEstimatedPose[i])
            .div(camresult.get().timestampSeconds - lastVisionEstimatedPoseTimestamp[i]);
        double deltav = visionVelMS.minus(chassisVelMS).getNorm();
        SmartDashboard.putString("vision vel ms (" + cameras.get(i).getName(),
            visionVelMS.toString());
        SmartDashboard.putNumber("velocity delta v(" + cameras.get(i).getName(), deltav);
        if ((chassisVelMS.getNorm() < 0.5 && deltav > STATIC_DELTAV_BOUND)
            || (chassisVelMS.getNorm() >= 0.5 && deltav > MOVING_DELTAV_BOUND)) {
          SmartDashboard.putString("Vision Update Status (" + cameras.get(i).getName() + ")",
              "VELOCITY_INCORRECT");
          continue;
        }
      }

      // calculate average distance to observed tags
      double avgdist = 0.0;
      int tagcnt = 0;
      for (var tgt : camresult.get().targetsUsed) {
        var tagpose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
        if (tagpose.isPresent()) {
          avgdist += tagpose.get().getTranslation()
              .getDistance(camresult.get().estimatedPose.getTranslation());
          tagcnt += 1;
        }
      }
      if (tagcnt > 0) {
        avgdist /= tagcnt;
      }

      // calculate update dev by distance
      // you find multitag results get smaller devs by dividing tag count here.
      double xyDev = xyDevFactors[i] * Math.pow(avgdist, 2) / tagcnt;
      double angleDev = Double.POSITIVE_INFINITY;
      // only update angle dev when using multitag
      if (camresult.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
        angleDev = angleDevFactors[i] * Math.pow(avgdist, 2) / tagcnt;
      }

      VisionObservation newobs = new VisionObservation(camresult.get(), xyDev, angleDev, cameras.get(i).getName());
      allObservation.add(newobs);
      SmartDashboard.putString("Vision Update Status (" + cameras.get(i).getName() + ")", "OK");
      SmartDashboard.putString("Vision Observation Log", newobs.toString() + " " + camresult.get().estimatedPose.toString());

      lastVisionEstimatedPose[i] = camresult.get().estimatedPose.getTranslation().toTranslation2d();
      lastVisionEstimatedPoseTimestamp[i] = camresult.get().timestampSeconds;
    }

    return allObservation;
  }

  private PoseStrategy determineStrategy(int camIdx, List<PhotonTrackedTarget> acceptableTargets) {
    double currentTime = Timer.getFPGATimestamp();
    boolean isMultiTag = acceptableTargets.size() > 1;
    // boolean isMultiTag = false;

    boolean useMultiTag = lstMultiTagDelayedBoolean.get(camIdx).update(currentTime, isMultiTag);

    if (useMultiTag) {
      return PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    } else {
      return PoseStrategy.LOWEST_AMBIGUITY;
    }
  }

  private void logOriginalTags(PhotonPipelineResult result, String cameraName) {
    String[] originalTagsInfo = new String[result.getTargets().size()];
    for (int i = 0; i < result.getTargets().size(); i++) {
      var target = result.getTargets().get(i);
      originalTagsInfo[i] = String.format(
          "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f", target.getFiducialId(),
          target.getArea(), target.getYaw(), target.getPitch(), target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Original Tags (" + cameraName + ")", originalTagsInfo);
  }

  private List<PhotonTrackedTarget> filterAcceptableTargets(PhotonPipelineResult result,
      double minArea) {
    return result.getTargets().stream().filter(target -> {
      boolean isAcceptable =
          target.getArea() > minArea && target.getPoseAmbiguity() < ACCEPTABLE_AMBIGUITY_THRESHOLD
              && Math.abs(target.getYaw()) < ACCEPTABLE_YAW_THRESHOLD
              && Math.abs(target.getPitch()) < ACCEPTABLE_PITCH_THRESHOLD;
      SmartDashboard.putString("Filtering Log",
          String.format(
              "ID: %d, Acceptable: %b, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
              target.getFiducialId(), isAcceptable, target.getArea(), target.getYaw(),
              target.getPitch(), target.getPoseAmbiguity()));
      return isAcceptable;
    }).collect(Collectors.toList());
  }

  private void logAcceptedTags(List<PhotonTrackedTarget> acceptableTargets, String cameraName) {
    String[] acceptedTagsInfo = new String[acceptableTargets.size()];
    for (int i = 0; i < acceptableTargets.size(); i++) {
      var target = acceptableTargets.get(i);
      acceptedTagsInfo[i] = String.format(
          "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f", target.getFiducialId(),
          target.getArea(), target.getYaw(), target.getPitch(), target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Accepted Tags (" + cameraName + ")", acceptedTagsInfo);
  }

  /**
   * Get the pose of an AprilTag from the field layout
   *
   * @param tagId The ID of the tag
   * @return The pose of the tag, if available
   */
  public Optional<Pose3d> getAprilTagPose(int tagId) {
    return aprilTagFieldLayout.getTagPose(tagId);
  }

  public PhotonPipelineResult getShooterSideLastResult() {
    return ApriltagCamShooterSide.getLatestResult();
  }
}
