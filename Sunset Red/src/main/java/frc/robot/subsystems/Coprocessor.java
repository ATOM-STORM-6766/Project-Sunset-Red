package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coprocessor extends SubsystemBase {
    private PhotonCamera ov9281 = new PhotonCamera("OV9281");
    private Transform3d kRobotToCamera = new Transform3d(0.25, 0.06, 0.25, new Rotation3d(0, 210.0/180*Math.PI, Math.PI));
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private Pose2d lastRobotPose = new Pose2d(); //get Pose from odom
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, ov9281, kRobotToCamera);

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic("RobotPose", Pose2d.struct).publish();

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    double lastTime = Timer.getFPGATimestamp();
    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> currRobotPose = getEstimatedGlobalPose(lastRobotPose);
        if(currRobotPose.isPresent()){
            publisher.set(currRobotPose.get().estimatedPose.toPose2d());
            lastRobotPose = currRobotPose.get().estimatedPose.toPose2d();
            SmartDashboard.putBoolean("photon_pose_present", true);
        }else{
            publisher.set(lastRobotPose);
            SmartDashboard.putBoolean("photon_pose_present", false);
        }


    }
}
