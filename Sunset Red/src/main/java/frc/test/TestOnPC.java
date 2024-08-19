package frc.test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionShootConstants;

public class TestOnPC {
    public static void main(String[] args) {

        // on blue side
        Translation2d position = new Translation2d(4.3, 6.3);
        Translation2d robotToGoal = position.minus(VisionShootConstants.kBlueSpeaker);

        System.out.println("dist: "+ robotToGoal + "m");
        System.out.println("heading" + robotToGoal.getAngle().getDegrees());
        System.out.println("Arm Angle:"+VisionShootConstants.kSpeakerAngleMap.get(robotToGoal.getNorm())+" deg");
        System.out.println("Shooter Speed:"+VisionShootConstants.kSpeakerRPSMap.get(robotToGoal.getNorm())+" rps");

    }
}
