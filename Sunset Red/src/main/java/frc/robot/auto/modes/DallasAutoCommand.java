package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.commands.NavTrapCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.TrapFan;
import frc.robot.utils.ShootingParameters;

public class DallasAutoCommand extends SequentialCommandGroup {
        // define path names
        private static final String kStartPathDallas = "Dallas StartPath";
        private static final String kShootPoseUnderStageTo52Path =
                        "Dallas ShootPoseUnderStage to 52";
        private static final String kShootPoseNearSideTo51Path = "Dallas ShootPoseNearSide to 51";
        private static final String kShootPoseUnderStageTo54Path = null;
        private static final String kShootPoseFarSideTo55Path = "Dallas ShootPoseFarSide to 55";
        private static final String kShootPoseUnderStageTo51Path =
                        "Dallas ShootPoseUnderStage to 51";
        private static final String kShootPoseNearSideTo52Path = "Dallas ShootPoseNearSide to 52";
        private static final String kShootPoseUnderStageTo55Path =
                        "Dallas ShootPoseUnderStage to 55";
        private static final String kShootPoseFarSideTo54Path = "Dallas ShootPoseFarSide to 54";

        // define shoot poses
        private static final Pose2d kShootPoseUnderStage =
                        new Pose2d(4.5, 4.63, Rotation2d.fromDegrees(-14.0));
        private static final Pose2d kShootPoseNearSide =
                        new Pose2d(4.5, 6.46, Rotation2d.fromDegrees(14.0));
        private static final Pose2d kShootPoseFarSide =
                        new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(28.0));

        // define shooting parameters
        private static final ShootingParameters kShootParamUnderStage =
                        new ShootingParameters(75, 32.5);
        private static final ShootingParameters kShootParamNearSide =
                        new ShootingParameters(75, 32.5);
        private static final ShootingParameters kShootParamFarSide = new ShootingParameters(75, 41);
        private static final ShootingParameters kShootParam32 = new ShootingParameters(75, 36.5);

        public enum DallasAutoStrategy {
                DROP53_GO_NEAR_SIDE, // drop 53 ahead, score for 52 51
                DROP53_GO_FAR_SIDE, // drop 53 ahead, score for 54 55
                WINGSHOT53_GO_NEAR_SIDE, // wing shot 53, score for 52 51
                WINGSHOT53_GO_FAR_SIDE, // wing shot 53, score for 54 55
                SCORE53_GO_NEAR_SIDE, // score 53, score for 52 51
                SCORE53_GO_NEAR_SIDE_FLIP, // score 53, score for 51 52
                SCORE53_GO_FAR_SIDE, // score 53, score for 54 55
                SCORE53_GO_FAR_SIDE_FLIP, // score 53, score for 55, 54
                TRAP53
        }

        public DallasAutoCommand(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
                        Transfer transfer, Intake intake, TrapFan trapFan,
                        DallasAutoStrategy strategy) {
                addCommands(AutoCommandFactory.buildPrepCommand(drivetrainSubsystem,
                                ShootingParameters.BELOW_SPEAKER, kStartPathDallas, arm, shooter,
                                transfer));

                switch (strategy) {
                        case DROP53_GO_NEAR_SIDE:
                                addCommands(buildDrop53GoForNearSide(drivetrainSubsystem, arm,
                                                shooter, transfer, intake));
                                break;
                        case DROP53_GO_FAR_SIDE:
                                addCommands(buildDrop53GoForFarSide(drivetrainSubsystem, arm,
                                                shooter, transfer, intake));
                                break;
                        case WINGSHOT53_GO_NEAR_SIDE:
                                addCommands(buildWingShot53GoForNearSide(drivetrainSubsystem, arm,
                                                shooter, transfer, intake));
                                break;
                        case WINGSHOT53_GO_FAR_SIDE:
                                addCommands(buildWingShot53GoForFarSide(drivetrainSubsystem, arm,
                                                shooter, transfer, intake));
                                break;
                        case SCORE53_GO_FAR_SIDE:
                                addCommands(buildScore53Routine(drivetrainSubsystem, arm, shooter,
                                                transfer, intake, false, false));
                                break;
                        case SCORE53_GO_FAR_SIDE_FLIP:
                                addCommands(buildScore53Routine(drivetrainSubsystem, arm, shooter,
                                                transfer, intake, false, true));
                                break;
                        case SCORE53_GO_NEAR_SIDE:
                                addCommands(buildScore53Routine(drivetrainSubsystem, arm, shooter,
                                                transfer, intake, true, false));
                                break;
                        case SCORE53_GO_NEAR_SIDE_FLIP:
                                addCommands(buildScore53Routine(drivetrainSubsystem, arm, shooter,
                                                transfer, intake, true, true));
                                break;
                        case TRAP53:
                                addCommands(buildTrap53(drivetrainSubsystem, arm, shooter, transfer,
                                                intake, trapFan));
                                break;

                }
        }

        /**
         * Build the command for the SCORE53 strategies (GO_FAR_SIDE, GO_FAR_SIDE_FLIP,
         * GO_NEAR_SIDE, GO_NEAR_SIDE_FLIP) Start at Dallas mid position, move to 53, shoot 32 along
         * the way Then go get and score two more game pieces based on the strategy
         * 
         * Fallback strategies: - For 53 intake: - Near side: go for 54 (-90 degree) - Far side: go
         * for 52 (90 degree) - For first note intake: - Near side: fallback is the second note -
         * Far side: fallback is the second note - For second note intake: - Near side: fallback is
         * the first note - Far side: fallback is the first note
         * 
         * The path to intake notes should activate chase note at appropriate positions: - Near
         * side: - First note (52): Pos 51.6 (can see 51 and 52, with 52 preferred) - Second note
         * (51): Pos 51.4 (can see 51 and 52, with 51 preferred) - Far side: - First note (54): Pos
         * 54.4 (can see 54 and 55, with 54 preferred) - Second note (55): Pos 54.6 (can see 54 and
         * 55, with 55 preferred)
         * 
         * @param drivetrainSubsystem
         * @param arm
         * @param shooter
         * @param transfer
         * @param intake
         * @param isNearSide
         * @param isStartWithOuterNotes
         * @return
         */
        private Command buildScore53Routine(DrivetrainSubsystem drivetrainSubsystem, Arm arm,
                        Shooter shooter, Transfer transfer, Intake intake, boolean isNearSide,
                        boolean isStartWithOuterNotes) {
                Pose2d firstShootPose = isNearSide ? kShootPoseNearSide : kShootPoseFarSide;
                ShootingParameters firstShootParam =
                                isNearSide ? kShootParamNearSide : kShootParamFarSide;

                String firstNotePath = isNearSide
                                ? (isStartWithOuterNotes ? kShootPoseUnderStageTo51Path
                                                : kShootPoseUnderStageTo52Path)
                                : (isStartWithOuterNotes ? kShootPoseUnderStageTo55Path
                                                : kShootPoseUnderStageTo54Path);

                String secondNotePath = isNearSide
                                ? (isStartWithOuterNotes ? kShootPoseNearSideTo52Path
                                                : kShootPoseNearSideTo51Path)
                                : (isStartWithOuterNotes ? kShootPoseFarSideTo54Path
                                                : kShootPoseFarSideTo55Path);

                Rotation2d firstNoteRotation = Rotation2d.fromDegrees(isStartWithOuterNotes ? -90 : 90);
                Rotation2d secondNoteRotation = Rotation2d.fromDegrees(isStartWithOuterNotes ? 90 : -90);

                Pose2d endChasePose = isNearSide ? FieldConstants.NOTE_54_POSITION
                                : FieldConstants.NOTE_55_POSITION;
                Rotation2d endChaseRotation = Rotation2d.fromDegrees(isNearSide ? -90 : 0);

                return new SequentialCommandGroup(
                                // Move to 53, intake and shoot 32
                                AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                                                drivetrainSubsystem, arm, shooter, transfer, intake,
                                                GamePieceProcessor.getInstance(), kStartPathDallas,
                                                kShootParam32,
                                                Rotation2d.fromDegrees(isNearSide ? -90 : 90)),

                                // Shoot 53
                                AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage,
                                                PathfindConstants.constraints)
                                                .deadlineWith(new SetArmAngleCommand(arm,
                                                                kShootParamUnderStage.angle_deg),
                                                                new SetShooterTargetCommand(shooter,
                                                                                kShootParamUnderStage.speed_rps)),

                                // Get first note
                                AutoCommandFactory.buildPathThenChaseNoteCommand(
                                                drivetrainSubsystem, arm, shooter, transfer, intake,
                                                GamePieceProcessor.getInstance(),
                                                AutoBuilder.followPath(PathPlannerPath
                                                                .fromPathFile(firstNotePath)),
                                                firstNoteRotation),

                                // Score first note
                                AutoBuilder.pathfindToPoseFlipped(firstShootPose,
                                                PathfindConstants.constraints)
                                                .deadlineWith(new SetArmAngleCommand(arm,
                                                                firstShootParam.angle_deg),
                                                                new SetShooterTargetCommand(shooter,
                                                                                firstShootParam.speed_rps)),

                                // Get second note
                                AutoCommandFactory.buildPathThenChaseNoteCommand(
                                                drivetrainSubsystem, arm, shooter, transfer, intake,
                                                GamePieceProcessor.getInstance(),
                                                AutoBuilder.followPath(PathPlannerPath
                                                                .fromPathFile(secondNotePath)),
                                                secondNoteRotation),

                                // Score second note
                                AutoBuilder.pathfindToPoseFlipped(firstShootPose,
                                                PathfindConstants.constraints),

                                // End chase
                                AutoCommandFactory.buildPathThenChaseNoteCommand(
                                                drivetrainSubsystem, arm, shooter, transfer, intake,
                                                GamePieceProcessor.getInstance(),
                                                AutoBuilder.pathfindToPoseFlipped(endChasePose,
                                                                PathfindConstants.constraints),
                                                endChaseRotation));
        }


        /**
         * Build the command for the TRAP53 strategy Start at Dallas mid position, move to 53, shoot
         * 32 along the way Then move to the nearest trap position
         * 
         * Fallback strategy for 53 intake: go for 54 (-90 degree)
         * 
         * @param drivetrainSubsystem
         * @param arm
         * @param shooter
         * @param transfer
         * @param intake
         * @param trapFan
         * @return
         */
        private Command buildTrap53(DrivetrainSubsystem drivetrainSubsystem, Arm arm,
                        Shooter shooter, Transfer transfer, Intake intake, TrapFan trapFan) {
                return new SequentialCommandGroup(
                                // Move to 53, intake and shoot 32 along the way, fallback for 53
                                // intake is 54
                                AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                                                drivetrainSubsystem, arm, shooter, transfer, intake,
                                                GamePieceProcessor.getInstance(), kStartPathDallas,
                                                kShootParam32, Rotation2d.fromDegrees(-90)),

                                // Move to the nearest trap position
                                new NavTrapCommand(drivetrainSubsystem, arm, shooter, intake,
                                                transfer, trapFan));
        }



        private Command buildWingShot53GoForFarSide(DrivetrainSubsystem drivetrainSubsystem,
                        Arm arm, Shooter shooter, Transfer transfer, Intake intake) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException(
                                "Unimplemented method 'buildWingShot53GoForFarSide'");
        }

        private Command buildWingShot53GoForNearSide(DrivetrainSubsystem drivetrainSubsystem,
                        Arm arm, Shooter shooter, Transfer transfer, Intake intake) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException(
                                "Unimplemented method 'buildWingShot53GoForNearSide'");
        }

        private Command buildDrop53GoForFarSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm,
                        Shooter shooter, Transfer transfer, Intake intake) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException(
                                "Unimplemented method 'buildDrop53GoForFarSide'");
        }

        private Command buildDrop53GoForNearSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm,
                        Shooter shooter, Transfer transfer, Intake intake) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException(
                                "Unimplemented method 'buildDrop53GoForNearSide'");
        }

}
