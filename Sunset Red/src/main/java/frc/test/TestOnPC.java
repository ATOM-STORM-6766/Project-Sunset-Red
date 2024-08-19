package frc.test;

import frc.robot.Constants.VisionShootConstants;
import frc.robot.auto.AutoRoutineConfig.AutoShootPositions;

public class TestOnPC {
    public static void main(String[] args) {
        double near_side_dist = AutoShootPositions.NEAR_SIDE.shootPose.getTranslation().minus(VisionShootConstants.kBlueSpeaker).getNorm();
        double far_side_dist = AutoShootPositions.FAR_SIDE.shootPose.getTranslation().minus(VisionShootConstants.kBlueSpeaker).getNorm();
        double under_stage = AutoShootPositions.UNDER_STAGE.shootPose.getTranslation().minus(VisionShootConstants.kBlueSpeaker).getNorm();

        System.out.println("dist: "+under_stage + "m");
        System.out.println("Arm Angle:"+VisionShootConstants.kSpeakerAngleMap.get(under_stage)+" deg");
        System.out.println("Shooter Speed:"+VisionShootConstants.kSpeakerRPSMap.get(under_stage)+" rps");
        
    }
}
