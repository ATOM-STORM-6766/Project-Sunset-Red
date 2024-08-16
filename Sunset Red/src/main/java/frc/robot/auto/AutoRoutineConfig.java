package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.ShootingParameters;

public class AutoRoutineConfig {
    public static class AutoShootingConfig {
        public final Pose2d shootPose;
        public final ShootingParameters shootParams;

        public AutoShootingConfig(Pose2d pose, ShootingParameters params) {
            this.shootPose = pose;
            this.shootParams = params;
        }
    }

    public static class AutoShootPositions {
        public static final AutoShootingConfig UNDER_STAGE =
                new AutoShootingConfig(new Pose2d(4.5, 4.63, Rotation2d.fromDegrees(-14.0)),
                        new ShootingParameters(75, 32.5));
        public static final AutoShootingConfig NEAR_SIDE =
                new AutoShootingConfig(new Pose2d(4.5, 6.46, Rotation2d.fromDegrees(14.0)),
                        new ShootingParameters(75, 32.5));
        public static final AutoShootingConfig FAR_SIDE = new AutoShootingConfig(
                new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(28.0)), new ShootingParameters(75, 41));
        public static final AutoShootingConfig NOTE_31 =
                new AutoShootingConfig(new Pose2d(2.85, 6.96, Rotation2d.fromDegrees(30.0)),
                        new ShootingParameters(75, 36.5)); // todo: tune this
        public static final AutoShootingConfig NOTE_32 =
                new AutoShootingConfig(null, new ShootingParameters(75, 36.5));
        public static final AutoShootingConfig NOTE_33 =
                new AutoShootingConfig(new Pose2d(2.65, 4.27, Rotation2d.fromDegrees(-30.0)),
                        new ShootingParameters(75, 36.5)); // todo: tune this
        public static final AutoShootingConfig DROP_53 =
                new AutoShootingConfig(null, new ShootingParameters(10, 36.5));
        public static final AutoShootingConfig HOME =
                new AutoShootingConfig(new Pose2d(1.47, 5.59, Rotation2d.fromDegrees(0)),
                        ShootingParameters.BELOW_SPEAKER);
    }

    public static class AutoPaths {
        public static final String START_DALLAS = "Dallas StartPath";
        public static final String START_CALIFORNIA = "California StartPath";
        public static final String FROM_53_TO_52 = "Dallas 53 to 52";
        public static final String FROM_53_TO_54 = "Dallas 53 to 54";
        public static final String FROM_54_TO_55 = "California 54 to 55";
        public static final String UNDER_STAGE_TO_51 = "ShootPoseUnderStage to 51";
        public static final String UNDER_STAGE_TO_52 = "ShootPoseUnderStage to 52";
        public static final String UNDER_STAGE_TO_54 = "ShootPoseUnderStage to 54";
        public static final String UNDER_STAGE_TO_55 = "ShootPoseUnderStage to 55";
        public static final String NEAR_SIDE_TO_51 = "ShootPoseNearSide to 51";
        public static final String NEAR_SIDE_TO_52 = "ShootPoseNearSide to 52";
        public static final String FAR_SIDE_TO_54 = "ShootPoseFarSide to 54";
        public static final String FAR_SIDE_TO_55 = "ShootPoseFarSide to 55";
        public static final String NEAR_SIDE_TO_53 = "ShootPoseNearSide to 53";
        public static final String FAR_SIDE_TO_53 = "ShootPoseFarSide to 53";
        public static final String HOME_TO_31 = "Dallas Home to 31";
        public static final String HOME_TO_33 = "Dallas Home to 33";
        public static final String FROM_52_TO_HOME = "Dallas 52 to Home";
        public static final String FROM_54_TO_HOME = "Dallas 54 to Home";
    }
}
