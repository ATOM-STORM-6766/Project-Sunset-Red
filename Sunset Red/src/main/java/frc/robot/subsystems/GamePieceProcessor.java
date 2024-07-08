package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GamePieceProcessor extends SubsystemBase {

  private static GamePieceProcessor mCoprocessor = new GamePieceProcessor();
  private PhotonCamera mUSBCamera = new PhotonCamera("rmoncam_720P");

  private Transform3d kRobotToPieceCam =
      new Transform3d(
          new Translation3d(-0.03, 0.0, 0.43),
          new Rotation3d(Math.toRadians(180.0), Math.toRadians(14.0), 0.0));

  public static GamePieceProcessor getInstance() {
    return mCoprocessor;
  }

  public GamePieceProcessor() {}

  public Optional<PhotonTrackedTarget> getClosestGamePieceInfo() {
    if (mUSBCamera.getLatestResult().hasTargets()) {
      return Optional.of(mUSBCamera.getLatestResult().getBestTarget());
    }
    return Optional.empty();
  }

  public Translation2d robotToPiece(PhotonTrackedTarget target) {
    double angle_rad = kRobotToPieceCam.getRotation().getY() + Math.toRadians(target.getPitch());
    double dist = kRobotToPieceCam.getTranslation().getZ() / Math.tan(angle_rad);

    Translation2d camToPiece = new Translation2d(dist, Rotation2d.fromDegrees(target.getYaw()));
    return kRobotToPieceCam.getTranslation().toTranslation2d().plus(camToPiece);
  }
}
