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
  private static final String SHOOTER_CAMERA_NAME = "TagCamShooterSide";
  private static final String INTAKE_CAMERA_NAME = "TagCamIntakeSide";
  private static final double ACCEPTABLE_AMBIGUITY_THRESHOLD = 0.15;
  private static final double MULTI_TAG_DELAY = 0.5;
  private static final double MIN_TARGET_AREA = 0.1;
  private static final double MAX_TARGET_YAW = 25;
  private static final double MAX_TARGET_PITCH = 25;
  private static final double MAX_VELOCITY_DELTA = 0.5;

  private final PhotonCamera ApriltagCamShooterSide;
  private final PhotonCamera ApriltagCamIntakeSide;
  private final PhotonPoseEstimator photonPoseEstimatorForShooterSide;
  private final PhotonPoseEstimator photonPoseEstimatorForIntakeSide;
  private final DualEdgeDelayedBoolean multiTagDelayedBoolean;
  private final Object poseLock = new Object();
  private final AprilTagFieldLayout aprilTagFieldLayout;

  private Translation2d lastVisionEstimatedPose = null;
  private double lastVisionEstimatedPoseTimestamp = 0;

  private final StructPublisher<Pose2d> lastRPpublisher;
  private final StructPublisher<Pose2d> currRPpublisher;
  private final StructPublisher<Pose3d> intakeSidePublisher;
  private final StructPublisher<Pose3d> shooterSidePublisher;

  private boolean loggingEnabled = true;

  private ApriltagCoprocessor() {
    try {
      ApriltagCamShooterSide = new PhotonCamera(SHOOTER_CAMERA_NAME);
      ApriltagCamIntakeSide = new PhotonCamera(INTAKE_CAMERA_NAME);

      Transform3d kRobotToCameraForShooterSide =
          new Transform3d(-0.28, -0.06, 0.25, new Rotation3d(0, 220.0 / 180 * Math.PI, 0));
      Transform3d kRobotToCameraForIntakeSide =
          new Transform3d(
              0.22,
              -0.075,
              0.08,
              new Rotation3d(
                  Units.degreesToRadians(180),
                  Units.degreesToRadians(-37),
                  Units.degreesToRadians(0)));

      aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

      photonPoseEstimatorForShooterSide =
          new PhotonPoseEstimator(
              aprilTagFieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              ApriltagCamShooterSide,
              kRobotToCameraForShooterSide);
      photonPoseEstimatorForIntakeSide =
          new PhotonPoseEstimator(
              aprilTagFieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              ApriltagCamIntakeSide,
              kRobotToCameraForIntakeSide);

      photonPoseEstimatorForShooterSide.setMultiTagFallbackStrategy(
          PoseStrategy.AVERAGE_BEST_TARGETS);
      photonPoseEstimatorForIntakeSide.setMultiTagFallbackStrategy(
          PoseStrategy.AVERAGE_BEST_TARGETS);

      double currentTime = Timer.getFPGATimestamp();
      multiTagDelayedBoolean =
          new DualEdgeDelayedBoolean(currentTime, MULTI_TAG_DELAY, EdgeType.RISING);

      lastRPpublisher =
          NetworkTableInstance.getDefault()
              .getTable("SmartDashboard")
              .getStructTopic("lastRobotPose", Pose2d.struct)
              .publish();
      currRPpublisher =
          NetworkTableInstance.getDefault()
              .getTable("SmartDashboard")
              .getStructTopic("currRobotPose", Pose2d.struct)
              .publish();
      intakeSidePublisher =
          NetworkTableInstance.getDefault()
              .getTable("SmartDashboard")
              .getStructTopic("Intake Side Estimated Pose", Pose3d.struct)
              .publish();
      shooterSidePublisher =
          NetworkTableInstance.getDefault()
              .getTable("SmartDashboard")
              .getStructTopic("Shooter Side Estimated Pose", Pose3d.struct)
              .publish();

    } catch (Exception e) {
      logToSmartDashboard("ApriltagCoprocessor Init Error", e.getMessage());
      throw new RuntimeException("Failed to initialize ApriltagCoprocessor", e);
    }
  }

  private static ApriltagCoprocessor instance = null;

  public static ApriltagCoprocessor getInstance() {
    if (instance == null) {
      instance = new ApriltagCoprocessor();
    }
    return instance;
  }

  /**
   * Update the estimated global pose of the robot using the latest vision results
   *
   * @param prevEstimatedRobotPose The previous estimated robot pose
   * @param chassisVelMS The velocity of the robot chassis in meters per second
   * @return The new estimated robot pose
   */
  public Optional<EstimatedRobotPose> updateEstimatedGlobalPose(
      Pose2d prevEstimatedRobotPose, Translation2d chassisVelMS) {
    Optional<EstimatedRobotPose> shooterSideResult =
        processCameraResult(
            ApriltagCamShooterSide, photonPoseEstimatorForShooterSide, prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> intakeSideResult =
        processCameraResult(
            ApriltagCamIntakeSide, photonPoseEstimatorForIntakeSide, prevEstimatedRobotPose);

    Optional<EstimatedRobotPose> bestResult = selectBestResult(shooterSideResult, intakeSideResult);

    synchronized (poseLock) {
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
        logToSmartDashboard("Vision Velocity (m/s)", visionVelMS.toString());
        double velocityDelta = visionVelMS.minus(chassisVelMS).getNorm();
        logToSmartDashboard("Velocity Delta (m/s)", velocityDelta);
        if (velocityDelta > MAX_VELOCITY_DELTA) {
          logToSmartDashboard("Vision Estimation Rejected", true);
          return Optional.empty();
        }
      }
      if (bestResult.isPresent()) {
        lastVisionEstimatedPose = bestResult.get().estimatedPose.getTranslation().toTranslation2d();
        lastVisionEstimatedPoseTimestamp = bestResult.get().timestampSeconds;
        logToSmartDashboard("Vision Estimation Rejected", false);
      }
    }
    return bestResult;
  }

  /**
   * Process the latest camera result and update the robot pose estimator
   *
   * @param camera The camera to process
   * @param photonPoseEstimator The pose estimator to update
   * @param prevEstimatedRobotPose The previous estimated robot pose
   * @return The new estimated robot pose, if successful
   */
  private Optional<EstimatedRobotPose> processCameraResult(
      PhotonCamera camera, PhotonPoseEstimator photonPoseEstimator, Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      logToSmartDashboard(
          "Vision Estimation Mode (" + camera.getName() + ")", "No Detected Targets");
      return Optional.empty();
    }

    logToSmartDashboard("Number of Targets (" + camera.getName() + ")", result.getTargets().size());
    logToSmartDashboard("Timestamp (" + camera.getName() + ")", result.getTimestampSeconds());

    logOriginalTags(result, camera.getName());

    var acceptableTargets = filterAcceptableTargets(result);
    logToSmartDashboard(
        "Number of Accepted Targets (" + camera.getName() + ")", acceptableTargets.size());

    logAcceptedTags(acceptableTargets, camera.getName());
    logTagAreas(acceptableTargets, camera.getName());

    if (acceptableTargets.isEmpty()) {
      logToSmartDashboard(
          "Vision Estimation Mode (" + camera.getName() + ")", "No Acceptable Targets");
      return Optional.empty();
    }

    PoseStrategy newStrategy = determineStrategy(acceptableTargets);
    photonPoseEstimator.setPrimaryStrategy(newStrategy);

    result.targets.clear();
    result.targets.addAll(acceptableTargets);

    Optional<EstimatedRobotPose> newEstimatedRobotPose = photonPoseEstimator.update(result);

    if (newEstimatedRobotPose.isPresent()) {
      logToSmartDashboard(
          "Vision Estimation Mode (" + camera.getName() + ")",
          newEstimatedRobotPose.get().strategy.toString());
      if (camera.getName().equals(SHOOTER_CAMERA_NAME)) {
        shooterSidePublisher.set(newEstimatedRobotPose.get().estimatedPose);
      } else {
        intakeSidePublisher.set(newEstimatedRobotPose.get().estimatedPose);
      }
    } else {
      logToSmartDashboard("Estimated Pose (" + camera.getName() + ")", "Failed to estimate pose");
    }

    return newEstimatedRobotPose;
  }

  /**
   * Log the total area of all tags in the camera view
   *
   * @param targets The list of targets
   * @param cameraName The name of the camera
   */
  private void logTagAreas(List<PhotonTrackedTarget> targets, String cameraName) {
    double totalArea = targets.stream().mapToDouble(PhotonTrackedTarget::getArea).sum();
    logToSmartDashboard("Total Tag Area (" + cameraName + ")", totalArea);
  }

  /**
   * Select the best result from the two camera sides
   *
   * @param shooterSideResult The result from the shooter side camera
   * @param intakeSideResult The result from the intake side camera
   * @return The best result from the two camera sides, if available
   */
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
  /**
   * Determine the strategy to use for pose estimation
   *
   * @param acceptableTargets The list of acceptable targets
   * @return The strategy to use
   */
  private PoseStrategy determineStrategy(List<PhotonTrackedTarget> acceptableTargets) {
    double currentTime = Timer.getFPGATimestamp();
    boolean isMultiTag = acceptableTargets.size() > 1;
    boolean useMultiTag = multiTagDelayedBoolean.update(currentTime, isMultiTag);
    return useMultiTag
        ? PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        : PoseStrategy.AVERAGE_BEST_TARGETS;
  }

  /**
   * Log the original tags detected by the camera
   *
   * @param result The camera result
   * @param cameraName The name of the camera
   */
  private void logOriginalTags(PhotonPipelineResult result, String cameraName) {
    String[] originalTagsInfo =
        result.getTargets().stream()
            .map(
                target ->
                    String.format(
                        "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
                        target.getFiducialId(),
                        target.getArea(),
                        target.getYaw(),
                        target.getPitch(),
                        target.getPoseAmbiguity()))
            .toArray(String[]::new);
    logToSmartDashboard("Original Tags (" + cameraName + ")", originalTagsInfo);
  }

  /**
   * Filter out targets that are not acceptable logic: 1. Area must be greater than MIN_TARGET_AREA
   * 2. Pose ambiguity must be less than ACCEPTABLE_AMBIGUITY_THRESHOLD 3. Yaw must be less than
   * MAX_TARGET_YAW 4. Pitch must be less than MAX_TARGET_PITCH
   *
   * @param result The camera result
   * @return The list of acceptable targets
   */
  private List<PhotonTrackedTarget> filterAcceptableTargets(PhotonPipelineResult result) {
    return result.getTargets().stream()
        .filter(this::isTargetAcceptable)
        .collect(Collectors.toList());
  }

  /**
   * Check if a target is acceptable
   *
   * @param target The target to check
   * @return True if the target is acceptable, false otherwise
   */
  private boolean isTargetAcceptable(PhotonTrackedTarget target) {
    boolean isAcceptable =
        target.getArea() > MIN_TARGET_AREA
            && target.getPoseAmbiguity() < ACCEPTABLE_AMBIGUITY_THRESHOLD
            && Math.abs(target.getYaw()) < MAX_TARGET_YAW
            && Math.abs(target.getPitch()) < MAX_TARGET_PITCH;

    logToSmartDashboard(
        "Target Filtering Log",
        String.format(
            "ID: %d, Acceptable: %b, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
            target.getFiducialId(),
            isAcceptable,
            target.getArea(),
            target.getYaw(),
            target.getPitch(),
            target.getPoseAmbiguity()));
    return isAcceptable;
  }

  /**
   * Log the accepted tags detected by the camera after filtering
   *
   * @param acceptableTargets The list of acceptable targets
   * @param cameraName The name of the camera
   */
  private void logAcceptedTags(List<PhotonTrackedTarget> acceptableTargets, String cameraName) {
    String[] acceptedTagsInfo =
        acceptableTargets.stream()
            .map(
                target ->
                    String.format(
                        "ID: %d, Area: %.3f, Yaw: %.2f, Pitch: %.2f, Ambiguity: %.3f",
                        target.getFiducialId(),
                        target.getArea(),
                        target.getYaw(),
                        target.getPitch(),
                        target.getPoseAmbiguity()))
            .toArray(String[]::new);
    logToSmartDashboard("Accepted Tags (" + cameraName + ")", acceptedTagsInfo);
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

  /**
   * Get the latest result from the shooter side camera
   *
   * @return The latest result
   */
  public PhotonPipelineResult getShooterSideLatestResult() {
    return ApriltagCamShooterSide.getLatestResult();
  }

  /**
   * Get the latest result from the intake side camera
   *
   * @return The latest result
   */
  public PhotonPipelineResult getIntakeSideLatestResult() {
    return ApriltagCamIntakeSide.getLatestResult();
  }

  /**
   * Reset the vision estimator This will clear the last vision estimated pose and timestamp use
   * this method for example when the robot is repositioned for a new match
   */
  public void resetVisionEstimator() {
    synchronized (poseLock) {
      lastVisionEstimatedPose = null;
      lastVisionEstimatedPoseTimestamp = 0;
    }
    logToSmartDashboard("Vision Estimation Reset", true);
  }

  /**
   * Check if any of the cameras sees any tags, also log the result to SmartDashboard
   *
   * @return True if any of the cameras sees tags, false otherwise
   */
  public boolean seesAnyTags() {
    boolean shooterSideSeesTags = ApriltagCamShooterSide.getLatestResult().hasTargets();
    boolean intakeSideSeesTags = ApriltagCamIntakeSide.getLatestResult().hasTargets();
    logToSmartDashboard("Shooter Side Sees Tags", shooterSideSeesTags);
    logToSmartDashboard("Intake Side Sees Tags", intakeSideSeesTags);
    return shooterSideSeesTags || intakeSideSeesTags;
  }

  /**
   * Log a value to SmartDashboard
   *
   * @param enabled True to enable logging, false to disable
   */
  public void setLoggingEnabled(boolean enabled) {
    loggingEnabled = enabled;
  }

  /**
   * Check if logging is enabled
   *
   * @return True if logging is enabled, false otherwise
   */
  public boolean isLoggingEnabled() {
    return loggingEnabled;
  }

  /**
   * Log a value to SmartDashboard
   *
   * @param key The key to log the value under
   * @param value The value to log
   */
  private void logToSmartDashboard(String key, Object value) {
    if (loggingEnabled) {
      if (value instanceof String) {
        SmartDashboard.putString(key, (String) value);
      } else if (value instanceof Double) {
        SmartDashboard.putNumber(key, (Double) value);
      } else if (value instanceof Boolean) {
        SmartDashboard.putBoolean(key, (Boolean) value);
      } else if (value instanceof String[]) {
        SmartDashboard.putStringArray(key, (String[]) value);
      } else if (value instanceof Integer) {
        SmartDashboard.putNumber(key, (Integer) value);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToSmartDashboard("Apriltag Coprocessor Active", true);
  }
}
