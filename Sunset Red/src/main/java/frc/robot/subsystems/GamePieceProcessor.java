package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class GamePieceProcessor extends SubsystemBase {

  private static GamePieceProcessor mCoprocessor = new GamePieceProcessor();
  private PhotonCamera mUSBCamera = new PhotonCamera("PieceCam");
  private double kLastDetectionTime = -100.0;
  private PhotonTrackedTarget kLastValidTarget = null;
  private final double kDetectionTimeout = 0.3;
  private Transform3d kRobotToPieceCam =
      new Transform3d(
          new Translation3d(-0.03, 0.0, 0.43),
          new Rotation3d(Math.toRadians(180.0), Math.toRadians(14.0), 0.0));

  public static GamePieceProcessor getInstance() {
    return mCoprocessor;
  }

  private GamePieceProcessor() {}

  public Optional<PhotonTrackedTarget> getClosestGamePieceInfo() {
    var result = mUSBCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    double currTime = Timer.getFPGATimestamp();
    if (hasTargets && (result.getTimestampSeconds() - kLastDetectionTime > 1e-6)) {
      updateLastValidTarget(result);
      return Optional.of(kLastValidTarget);
    } else if (currTime - kLastDetectionTime < kDetectionTimeout) {
      return Optional.ofNullable(kLastValidTarget);
    }
    return Optional.empty();
  }

  private void updateLastValidTarget(PhotonPipelineResult result) {
    kLastValidTarget = result.getBestTarget();
    kLastDetectionTime = result.getTimestampSeconds();
  }

  public Translation2d robotToPiece(PhotonTrackedTarget target) {
    if (target == null) {
      return new Translation2d(); // Return a default position if target is null
    }

    // Constants
    final double NOTE_OUTER_DIAMETER_METERS = 14 * 0.0254; // 14 inches to meters
    final double CAMERA_FOV_HORIZONTAL = Math.toRadians(66); // Adjust based on your camera's FOV
    final int CAMERA_RESOLUTION_WIDTH = 640; // Adjust based on your camera's resolution

    // Get the bounding box width in pixels
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    double widthPixels = 0;
    if (corners != null && corners.size() >= 2) {
      widthPixels = Math.abs(corners.get(1).x - corners.get(0).x);
    }

    // If widthPixels is 0, return a default position to avoid division by zero
    if (widthPixels == 0) {
      return new Translation2d();
    }

    // Calculate the distance based on the apparent size of the Note
    double distanceEstimate =
        (NOTE_OUTER_DIAMETER_METERS * CAMERA_RESOLUTION_WIDTH)
            / (2 * widthPixels * Math.tan(CAMERA_FOV_HORIZONTAL / 2));

    // Get the yaw and pitch from the target
    double yawRadians = Math.toRadians(target.getYaw());
    double pitchRadians = Math.toRadians(target.getPitch());

    // Calculate x, y, z coordinates relative to the camera
    double x = distanceEstimate * Math.cos(pitchRadians) * Math.sin(yawRadians);
    double y = distanceEstimate * Math.cos(pitchRadians) * Math.cos(yawRadians);
    double z = distanceEstimate * Math.sin(pitchRadians);

    // Create a Translation3d for the piece relative to the camera
    Translation3d camToPiece3d = new Translation3d(y, -x, -z);

    // Apply the robot-to-camera transform to get the piece position in robot frame
    Translation3d robotToPiece3d =
        kRobotToPieceCam.plus(new Transform3d(camToPiece3d, new Rotation3d())).getTranslation();

    // Return the 2D translation (ignoring height)
    return new Translation2d(robotToPiece3d.getX(), robotToPiece3d.getY());
  }

  @Override
  public void periodic() {
    var result = mUSBCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    SmartDashboard.putBoolean("PieceCam/HasTarget", hasTargets);
    SmartDashboard.putNumber("PieceCam/LatencyMillis", result.getLatencyMillis());

    if (hasTargets && (result.getTimestampSeconds() - kLastDetectionTime > 1e-6)) {
      var target = result.getBestTarget();
      if (target != null) {
        processValidTarget(target);
        updateLastValidTarget(result);
      } else {
        clearDashboardValues();
      }
    } else if (Timer.getFPGATimestamp() - kLastDetectionTime < kDetectionTimeout) {
      if (kLastValidTarget != null) {
        processValidTarget(kLastValidTarget);
      }
    } else {
      clearDashboardValues();
    }
  }

  private void processValidTarget(PhotonTrackedTarget target) {
    var piecePosition = robotToPiece(target);

    // Calculate distance using the 2D position
    double distance = piecePosition.getNorm();

    // Put values on SmartDashboard for verification
    SmartDashboard.putNumber("PieceCam/Distance", distance);
    SmartDashboard.putNumber("PieceCam/Piece to robot x", piecePosition.getX());
    SmartDashboard.putNumber("PieceCam/Piece to robot y", piecePosition.getY());

    SmartDashboard.putNumber("PieceCam/X", piecePosition.getX());
    SmartDashboard.putNumber("PieceCam/Y", piecePosition.getY());
    SmartDashboard.putNumber("PieceCam/Yaw", target.getYaw());
    SmartDashboard.putNumber("PieceCam/Pitch", target.getPitch());
    SmartDashboard.putNumber("PieceCam/Area", target.getArea());

    // Get bounding box width for verification
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners != null && corners.size() >= 2) {
      double widthPixels = Math.abs(corners.get(1).x - corners.get(0).x);
      SmartDashboard.putNumber("PieceCam/BoundingBoxWidth", widthPixels);
    } else {
      SmartDashboard.putNumber("PieceCam/BoundingBoxWidth", 0);
    }
  }

  private void clearDashboardValues() {
    SmartDashboard.putNumber("PieceCam/Distance", -1);
    SmartDashboard.putNumber("PieceCam/X", 0);
    SmartDashboard.putNumber("PieceCam/Y", 0);
    SmartDashboard.putNumber("PieceCam/Yaw", 0);
    SmartDashboard.putNumber("PieceCam/Pitch", 0);
    SmartDashboard.putNumber("PieceCam/Area", 0);
    SmartDashboard.putNumber("PieceCam/BoundingBoxWidth", 0);
  }
}
