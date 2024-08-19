package frc.robot.auto;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.ShootingParameters;

public class AutoRoutineConfig {
        public static class AutoShootingConfig {
                public final Pose2d shootPose;
                public final ShootingParameters shootParams;
                public final PathPlannerPath approachShootPosePath;

                public AutoShootingConfig(Pose2d pose, ShootingParameters params,
                                PathPlannerPath approachShootPosePath) {
                        this.shootPose = pose;
                        this.shootParams = params;
                        this.approachShootPosePath = approachShootPosePath;
                }
        }

        public static class AutoShootPositions {
                public static final AutoShootingConfig UNDER_STAGE = new AutoShootingConfig(
                                new Pose2d(4.5, 4.63, Rotation2d.fromDegrees(-14.0)),
                                new ShootingParameters(75, 30.93), AutoPaths.APPROACH_UNDER_STAGE);
                public static final AutoShootingConfig NEAR_SIDE = new AutoShootingConfig(
                                new Pose2d(3.8, 6.30, Rotation2d.fromDegrees(-167.94921511659044 + 180)),
                                new ShootingParameters(70.98022760533058, 33.07934067924468),
                                AutoPaths.APPROACH_NEAR_SIDE);
                public static final AutoShootingConfig FAR_SIDE = new AutoShootingConfig(
                                new Pose2d(3.45, 3.0, Rotation2d.fromDegrees(-28.0)), new ShootingParameters(75, 33.4),
                                AutoPaths.APPROACH_FAR_SIDE);
                public static final AutoShootingConfig NOTE_31 = new AutoShootingConfig(
                                new Pose2d(2.85, 6.96, Rotation2d.fromDegrees(30.0)),
                                new ShootingParameters(75, 36.5), null); // todo: tune this
                public static final AutoShootingConfig NOTE_32 = new AutoShootingConfig(null,
                                new ShootingParameters(75, 34), null);
                public static final AutoShootingConfig NOTE_33 = new AutoShootingConfig(
                                new Pose2d(2.65, 4.27, Rotation2d.fromDegrees(-30.0)),
                                new ShootingParameters(75, 36.5), null); // todo: tune this
                public static final AutoShootingConfig DROP_53 = new AutoShootingConfig(null,
                                new ShootingParameters(10, 36.5), null);
                public static final AutoShootingConfig HOME = new AutoShootingConfig(
                                new Pose2d(1.47, 5.59, Rotation2d.fromDegrees(0)),
                                ShootingParameters.BELOW_SPEAKER, null);
        }

        public static class AutoPaths {
                public static final PathPlannerPath START_DALLAS = PathPlannerPath.fromPathFile("Dallas StartPath");
                public static final PathPlannerPath START_CALIFORNIA = PathPlannerPath
                                .fromPathFile("California StartPath");
                public static final PathPlannerPath START_ARIZONA_NEAR = PathPlannerPath
                                .fromPathFile("Arizona StartPath Near");
                public static final PathPlannerPath START_ARIZONA_FAR = PathPlannerPath
                                .fromPathFile("Arizona StartPath Far");
                public static final PathPlannerPath FROM_53_TO_52 = PathPlannerPath.fromPathFile("Dallas 53 to 52");
                public static final PathPlannerPath FROM_53_TO_54 = PathPlannerPath.fromPathFile("Dallas 53 to 54");
                public static final PathPlannerPath FROM_54_TO_55 = PathPlannerPath.fromPathFile("California 54 to 55");
                public static final PathPlannerPath CALI_53_TO_54 = PathPlannerPath.fromPathFile("California 53 to 54");
                public static final PathPlannerPath UNDER_STAGE_TO_51 = PathPlannerPath
                                .fromPathFile("ShootPoseUnderStage to 51");
                public static final PathPlannerPath UNDER_STAGE_TO_52 = PathPlannerPath
                                .fromPathFile("ShootPoseUnderStage to 52");
                public static final PathPlannerPath UNDER_STAGE_TO_53 = PathPlannerPath
                                .fromPathFile("ShootPoseUnderStage to 53");
                public static final PathPlannerPath UNDER_STAGE_TO_54 = PathPlannerPath
                                .fromPathFile("ShootPoseUnderStage to 54");
                public static final PathPlannerPath UNDER_STAGE_TO_55 = PathPlannerPath
                                .fromPathFile("ShootPoseUnderStage to 55");
                public static final PathPlannerPath NEAR_SIDE_TO_51 = PathPlannerPath
                                .fromPathFile("ShootPoseNearSide to 51");
                public static final PathPlannerPath NEAR_SIDE_TO_52 = PathPlannerPath
                                .fromPathFile("ShootPoseNearSide to 52");
                public static final PathPlannerPath FAR_SIDE_TO_54 = PathPlannerPath
                                .fromPathFile("ShootPoseFarSide to 54");
                public static final PathPlannerPath FAR_SIDE_TO_55 = PathPlannerPath
                                .fromPathFile("ShootPoseFarSide to 55");
                public static final PathPlannerPath NEAR_SIDE_TO_53 = PathPlannerPath
                                .fromPathFile("ShootPoseNearSide to 53");
                public static final PathPlannerPath FAR_SIDE_TO_53 = PathPlannerPath
                                .fromPathFile("ShootPoseFarSide to 53");
                public static final PathPlannerPath HOME_TO_31 = PathPlannerPath.fromPathFile("Dallas Home to 31");
                public static final PathPlannerPath HOME_TO_33 = PathPlannerPath.fromPathFile("Dallas Home to 33");
                public static final PathPlannerPath FROM_52_TO_HOME = PathPlannerPath.fromPathFile("Dallas 52 to Home");
                public static final PathPlannerPath FROM_54_TO_HOME = PathPlannerPath.fromPathFile("Dallas 54 to Home");
                public static final PathPlannerPath APPROACH_UNDER_STAGE = PathPlannerPath
                                .fromPathFile("Approach ShootPoseUnderStage");
                public static final PathPlannerPath APPROACH_NEAR_SIDE = PathPlannerPath
                                .fromPathFile("Approach ShootPoseNearSide");
                public static final PathPlannerPath APPROACH_FAR_SIDE = PathPlannerPath
                                .fromPathFile("Approach ShootPoseFarSide");
        }
}
