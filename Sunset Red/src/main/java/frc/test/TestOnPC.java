package frc.test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionShootConstants;
import frc.robot.auto.AutoRoutineConfig.AutoShootPositions;

public class TestOnPC {
    public static void main(String[] args) {

        // on blue side
        Translation2d near_side = AutoShootPositions.NEAR_SIDE.shootPose.getTranslation().minus(VisionShootConstants.kBlueSpeaker);
        Translation2d far_side = AutoShootPositions.FAR_SIDE.shootPose.getTranslation().minus(VisionShootConstants.kBlueSpeaker);
        Translation2d under_stage = AutoShootPositions.UNDER_STAGE.shootPose.getTranslation().minus(VisionShootConstants.kBlueSpeaker);

        Translation2d robotToGoal = near_side;

        System.out.println("dist: "+ robotToGoal + "m");
        System.out.println("heading" + robotToGoal.unaryMinus().getAngle().getDegrees());
        System.out.println("Arm Angle:"+VisionShootConstants.kSpeakerAngleMap.get(robotToGoal.getNorm())+" deg");
        System.out.println("Shooter Speed:"+VisionShootConstants.kSpeakerRPSMap.get(robotToGoal.getNorm())+" rps");

    }
}
