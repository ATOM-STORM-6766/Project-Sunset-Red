package frc.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionShootConstants;

public class TestOnPC {
    public static void main(String[] args) {
        // Calc Vision Shoot Param
        Translation2d shootTranslation =  new Translation2d(3.8, 6.30);
        Translation2d robotToGoal = VisionShootConstants.kBlueSpeaker.minus(shootTranslation);
        System.out.println("heading" + robotToGoal.getAngle().getDegrees());
        System.out.println("arm angle: " + VisionShootConstants.kSpeakerAngleMap.get(robotToGoal.getNorm()));
        System.out.println("shooter rps" + VisionShootConstants.kSpeakerRPSMap.get(robotToGoal.getNorm()));
    }
}
