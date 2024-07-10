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
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ApriltagCoprocessor extends SubsystemBase {
  private static ApriltagCoprocessor mCoprocessor = new ApriltagCoprocessor();
  private static final double ACCEPTABLE_AMBIGUITY_THRESHOLD = 0.15;
  private DualEdgeDelayedBoolean multiTagDelayedBoolean;
  private static final double MULTI_TAG_DELAY = 0.5;

  public static ApriltagCoprocessor getInstance() {
    return mCoprocessor;
  }

  private ApriltagCoprocessor() {
    photonPoseEstimatorForShooterSide.setMultiTagFallbackStrategy(
        PoseStrategy.AVERAGE_BEST_TARGETS);
    photonPoseEstimatorForIntakeSide.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
    double currentTime = Timer.getFPGATimestamp();
    multiTagDelayedBoolean =
        new DualEdgeDelayedBoolean(currentTime, MULTI_TAG_DELAY, EdgeType.RISING);
  }

  private PhotonCamera ApriltagCamShooterSide = new PhotonCamera("TagCamShooterSide");
  private PhotonCamera ApriltagCamIntakeSide = new PhotonCamera("TagCamIntakeSide");

  private Transform3d kRobotToCameraForShooterSide =
      new Transform3d(-0.28, -0.06, 0.25, new Rotation3d(0, 220.0 / 180 * Math.PI, 0));

  private Transform3d kRobotToCameraForIntakeSide =
      new Transform3d(
          0.22,
          -0.075,
          0.08,
          new Rotation3d(
              Units.degreesToRadians(180), Units.degreesToRadians(-37), Units.degreesToRadians(0)));

  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  PhotonPoseEstimator photonPoseEstimatorForShooterSide =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          ApriltagCamShooterSide,
          kRobotToCameraForShooterSide);
  PhotonPoseEstimator photonPoseEstimatorForIntakeSide =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          ApriltagCamIntakeSide,
          kRobotToCameraForIntakeSide);

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

  StructPublisher<Pose3d> intakeSidePublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("Intake Side Estimated Pose", Pose3d.struct)
          .publish();

  StructPublisher<Pose3d> shooterSidePublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("Shooter Side Estimated Pose", Pose3d.struct)
          .publish();

  private Translation2d lastVisionEstimatedPose = null;
  private double lastVisionEstimatedPoseTimestamp = 0;

  private Optional<EstimatedRobotPose> processCameraResult(
      PhotonCamera camera, PhotonPoseEstimator photonPoseEstimator, Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      SmartDashboard.putString(
          "Vision Estimation Mode (" + camera.getName() + ")", "No Detected Targets");
      return Optional.empty();
    }

    SmartDashboard.putNumber(
        "Number of Targets (" + camera.getName() + ")", result.getTargets().size());
    SmartDashboard.putNumber("Timestamp (" + camera.getName() + ")", result.getTimestampSeconds());

    logOriginalTags(result, camera.getName());

    var acceptableTargets = filterAcceptableTargets(result);
    SmartDashboard.putNumber(
        "Number of Accepted Targets (" + camera.getName() + ")", acceptableTargets.size());

    logAcceptedTags(acceptableTargets, camera.getName());
    logTagAreas(acceptableTargets, camera.getName()); // Log tag areas

    if (acceptableTargets.isEmpty()) {
      SmartDashboard.putString(
          "Vision Estimation Mode (" + camera.getName() + ")", "No Acceptable Targets");
      return Optional.empty();
    }

    PoseStrategy newStrategy = determineStrategy(acceptableTargets);
    photonPoseEstimator.setPrimaryStrategy(newStrategy);

    result.targets.clear();
    result.targets.addAll(acceptableTargets);

    Optional<EstimatedRobotPose> newEstimatedRobotPose = photonPoseEstimator.update(result);

    if (newEstimatedRobotPose.isPresent()) {
      SmartDashboard.putString(
          "Vision Estimation Mode (" + camera.getName() + ")",
          newEstimatedRobotPose.get().strategy.toString());
      if (camera.getName() == "TagCamShooterSide") {
        shooterSidePublisher.set(newEstimatedRobotPose.get().estimatedPose);
      } else {
        intakeSidePublisher.set(newEstimatedRobotPose.get().estimatedPose);
      }
    } else {
      SmartDashboard.putStringArray(
          "Estimated Pose (" + camera.getName() + ")", new String[] {"Failed to estimate pose"});
    }

    return newEstimatedRobotPose;
  }

  private void logTagAreas(List<PhotonTrackedTarget> targets, String cameraName) {
    double totalArea = targets.stream().mapToDouble(PhotonTrackedTarget::getArea).sum();
    SmartDashboard.putNumber("Total Tag Area (" + cameraName + ")", totalArea);
  }

  private Optional<EstimatedRobotPose> selectBestResult(
      Optional<EstimatedRobotPose> shooterSideResult,
      Optional<EstimatedRobotPose> intakeSideResult) {
    if (shooterSideResult.isPresent() && intakeSideResult.isPresent()) {
      PoseStrategy shooterStrategy = shooterSideResult.get().strategy;
      PoseStrategy intakeStrategy = intakeSideResult.get().strategy;

      if (shooterStrategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
          && intakeStrategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
        return shooterSideResult;
      } else if (intakeStrategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
          && shooterStrategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
        return intakeSideResult;
      } else {
        double shooterArea =
            SmartDashboard.getNumber(
                "Total Tag Area (" + ApriltagCamShooterSide.getName() + ")", 0);
        double intakeArea =
            SmartDashboard.getNumber("Total Tag Area (" + ApriltagCamIntakeSide.getName() + ")", 0);
        return shooterArea > intakeArea ? shooterSideResult : intakeSideResult;
      }
    } else if (shooterSideResult.isPresent()) {
      return shooterSideResult;
    } else if (intakeSideResult.isPresent()) {
      return intakeSideResult;
    } else {
      return Optional.empty();
    }
  }

  public Optional<EstimatedRobotPose> updateEstimatedGlobalPose(
      Pose2d prevEstimatedRobotPose, Translation2d chassisVelMS) {
    Optional<EstimatedRobotPose> shooterSideResult =
        processCameraResult(
            ApriltagCamShooterSide, photonPoseEstimatorForShooterSide, prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> intakeSideResult =
        processCameraResult(
            ApriltagCamIntakeSide, photonPoseEstimatorForIntakeSide, prevEstimatedRobotPose);

    Optional<EstimatedRobotPose> bestResult = selectBestResult(shooterSideResult, intakeSideResult);

    var return_pose = bestResult;

    if (lastVisionEstimatedPose != null
        && bestResult.isPresent()
        && lastVisionEstimatedPoseTimestamp != bestResult.get().timestampSeconds) {
      Translation2d visionVelMS =
          bestResult
              .get()
              .estimatedPose
              .getTranslation()
              .toTranslation2d()
              .minus(lastVisionEstimatedPose)
              .div(bestResult.get().timestampSeconds - lastVisionEstimatedPoseTimestamp);
      SmartDashboard.putString("vision vel ms", visionVelMS.toString());
      SmartDashboard.putNumber("velocity delta v", visionVelMS.minus(chassisVelMS).getNorm());
      if (visionVelMS.minus(chassisVelMS).getNorm() > 0.5) {
        return_pose = Optional.empty();
      }
    }

    if (bestResult.isPresent()) {
      lastVisionEstimatedPose = bestResult.get().estimatedPose.getTranslation().toTranslation2d();
      lastVisionEstimatedPoseTimestamp = bestResult.get().timestampSeconds;
    }

    return return_pose;
  }

  private PoseStrategy determineStrategy(List<PhotonTrackedTarget> acceptableTargets) {
    double currentTime = Timer.getFPGATimestamp();
    boolean isMultiTag = acceptableTargets.size() > 1;

    boolean useMultiTag = multiTagDelayedBoolean.update(currentTime, isMultiTag);

    if (useMultiTag) {
      return PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    } else {
      return PoseStrategy.AVERAGE_BEST_TARGETS;
    }
  }

  private void logOriginalTags(PhotonPipelineResult result, String cameraName) {
    String[] originalTagsInfo = new String[result.getTargets().size()];
    for (int i = 0; i < result.getTargets().size(); i++) {
      var target = result.getTargets().get(i);
      originalTagsInfo[i] =
          String.format(
              "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
              target.getFiducialId(),
              target.getArea(),
              target.getYaw(),
              target.getPitch(),
              target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Original Tags (" + cameraName + ")", originalTagsInfo);
  }

  private List<PhotonTrackedTarget> filterAcceptableTargets(PhotonPipelineResult result) {
    return result.getTargets().stream()
        .filter(
            target -> {
              boolean isAcceptable =
                  target.getArea() > 0.07
                      && target.getPoseAmbiguity() < ACCEPTABLE_AMBIGUITY_THRESHOLD
                      && Math.abs(target.getYaw()) < 30
                      && Math.abs(target.getPitch()) < 25;
              SmartDashboard.putString(
                  "Filtering Log",
                  String.format(
                      "ID: %d, Acceptable: %b, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
                      target.getFiducialId(),
                      isAcceptable,
                      target.getArea(),
                      target.getYaw(),
                      target.getPitch(),
                      target.getPoseAmbiguity()));
              return isAcceptable;
            })
        .collect(Collectors.toList());
  }

  private void logAcceptedTags(List<PhotonTrackedTarget> acceptableTargets, String cameraName) {
    String[] acceptedTagsInfo = new String[acceptableTargets.size()];
    for (int i = 0; i < acceptableTargets.size(); i++) {
      var target = acceptableTargets.get(i);
      acceptedTagsInfo[i] =
          String.format(
              "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
              target.getFiducialId(),
              target.getArea(),
              target.getYaw(),
              target.getPitch(),
              target.getPoseAmbiguity());
    }
    SmartDashboard.putStringArray("Accepted Tags (" + cameraName + ")", acceptedTagsInfo);
  }
}
