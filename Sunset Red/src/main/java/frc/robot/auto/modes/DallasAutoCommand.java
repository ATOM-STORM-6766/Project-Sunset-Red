package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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
    private static final String kShootPoseUnderStageTo52Path = "Dallas ShootPoseUnderStage to 52";
    private static final String kShootPoseNearSideTo51Path = "Dallas ShootPoseNearSide to 51";
    private static final String kShootPoseUnderStageTo54Path = null;
    private static final String kShootPoseFarSideTo55Path = "Dallas ShootPoseFarSide to 55";

    // define shoot poses
    private static final Pose2d kShootPoseUnderStage = new Pose2d(4.5, 4.63, Rotation2d.fromDegrees(-14.0));
    private static final Pose2d kShootPoseNearSide = new Pose2d(4.5, 6.46, Rotation2d.fromDegrees(14.0));
    private static final Pose2d kShootPoseFarSide = new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(28.0));

    // define shooting parameters
    private static final ShootingParameters kShootParamUnderStage = new ShootingParameters(75, 32);
    private static final ShootingParameters kShootParamNearSide = new ShootingParameters(75, 32);
    private static final ShootingParameters kShootParamFarSide = new ShootingParameters(75, 31);
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

    public DallasAutoCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Arm arm,
            Shooter shooter,
            Transfer transfer,
            Intake intake,
            TrapFan trapFan,
            DallasAutoStrategy strategy) {
        addCommands(
                AutoCommandFactory.buildPrepCommand(
                        drivetrainSubsystem,
                        ShootingParameters.BELOW_SPEAKER,
                        kStartPathDallas,
                        arm,
                        shooter,
                        transfer));

        switch (strategy) {
            case DROP53_GO_NEAR_SIDE:
                addCommands(buildDrop53GoForNearSide(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case DROP53_GO_FAR_SIDE:
                addCommands(buildDrop53GoForFarSide(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case WINGSHOT53_GO_NEAR_SIDE:
                addCommands(buildWingShot53GoForNearSide(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case WINGSHOT53_GO_FAR_SIDE:
                addCommands(buildWingShot53GoForFarSide(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case SCORE53_GO_NEAR_SIDE:
                addCommands(buildScore53GoForNearSide(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case SCORE53_GO_NEAR_SIDE_FLIP:
                addCommands(buildScore53GoForNearSideFlip(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case SCORE53_GO_FAR_SIDE:
                addCommands(buildScore53GoForFarSide(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case SCORE53_GO_FAR_SIDE_FLIP:
                addCommands(buildScore53GoForFarSideFlip(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case TRAP53:
                addCommands(buildTrap53(drivetrainSubsystem, arm, shooter, transfer, intake, trapFan));
                break;

        }
    }

    private Command buildScore53GoForFarSideFlip(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'buildScore53GoForFarSideFlip'");
    }

    private Command buildScore53GoForNearSideFlip(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'buildScore53GoForNearSideFlip'");
    }

    /**
     * Build the command for the TRAP53 strategy
     * Start at Dallas mid position, move to 53, shoot 32 along the way
     * Then move to the nearest trap position
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
    private Command buildTrap53(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter, Transfer transfer,
            Intake intake, TrapFan trapFan) {
        return new SequentialCommandGroup(
                // Move to 53, intake and shoot 32 along the way, fallback for 53 intake is 54
                AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                        drivetrainSubsystem,
                        arm,
                        shooter,
                        transfer,
                        intake,
                        GamePieceProcessor.getInstance(),
                        kStartPathDallas,
                        kShootParam32,
                        Rotation2d.fromDegrees(-90)),

                // Move to the nearest trap position
                new NavTrapCommand(drivetrainSubsystem, arm, shooter, intake, transfer, trapFan));
    }

    /**
     * Build the command for the SCORE53_GO_FAR_SIDE strategy
     * Start at Dallas mid position, move to 53, shoot 32 along the way
     * Then go get 54, score 54, then go get 55, score 55
     * 
     * Fallback strategy for 53 intake: go for 52 (90 degree), reason is that we
     * need 54 and 55 later
     * The path to intake 54 should activate chase note at Pos 54.4 (A position that
     * can see 54 and 55, with 54 preferred)
     * The path to intake 55 should activate chase note at Pos 54.6 (A position that
     * can see 54 and 55, with 55 preferred)
     * 
     * @param drivetrainSubsystem
     * @param arm
     * @param shooter
     * @param transfer
     * @param intake
     * @return
     */
    private Command buildScore53GoForFarSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        return new SequentialCommandGroup(
                // Move to 53, intaking and shooting 32 on the way, fallback for 53 intake is 52
                AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                        drivetrainSubsystem,
                        arm,
                        shooter,
                        transfer,
                        intake,
                        GamePieceProcessor.getInstance(),
                        kStartPathDallas,
                        kShootParam32,
                        Rotation2d.fromDegrees(90)),
                // Shoot 53, use pathfindToPose flipped to go to shooting position, shooting
                // position for 53 is under stage
                AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                        .deadlineWith(
                                new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                                new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)),
                // Get 54, use Pathfollow to go to 54 position, fallback for 54 intake is 55
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake,
                        GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(kShootPoseUnderStageTo54Path)),
                        Rotation2d.fromDegrees(-90)),
                // Score 54, use pathfindToPose flipped to go to shooting position
                AutoBuilder.pathfindToPoseFlipped(kShootPoseFarSide, PathfindConstants.constraints)
                        .deadlineWith(
                                new SetArmAngleCommand(arm, kShootParamFarSide.angle_deg),
                                new SetShooterTargetCommand(shooter, kShootParamFarSide.speed_rps)),
                // Get 55, use Pathfollow to go to 55 position, fallback for 55 intake is 54
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake,
                        GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(kShootPoseFarSideTo55Path)),
                        Rotation2d.fromDegrees(90)),
                // Score 55, use pathfindToPose flipped to go to shooting position
                AutoBuilder.pathfindToPoseFlipped(kShootPoseFarSide, PathfindConstants.constraints),

                // End, for now we just chase 55, it should be stopping at the midline
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake,
                        GamePieceProcessor.getInstance(), AutoBuilder
                                .pathfindToPoseFlipped(FieldConstants.NOTE_55_POSITION, PathfindConstants.constraints),
                        Rotation2d.fromDegrees(0)));

    }

    /**
     * Build the command for the SCORE53_GO_NEAR_SIDE strategy
     * Start at Dallas mid position, move to 53, shoot 32 along the way
     * Then go get 52, score 52, then go get 51, score 51
     * 
     * Fallback strategy for 53 intake: go for 54 (-90 degree), reason is that we
     * need 51 and 52 later
     * The path to intake 52 should activate chase note at Pos 51.6 (A position that
     * can see 51 and 52, with 52 preferred)\
     * The path to intake 51 should activate chase note at Pos 51.4 (A position that
     * can see 51 and 52, with 51 preferred)
     * 
     * @param drivetrainSubsystem
     * @param arm
     * @param shooter
     * @param transfer
     * @param intake
     * @return
     */
    private Command buildScore53GoForNearSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        return new SequentialCommandGroup(
                // Move to 53, intaking and shooting 32 on the way, fallback for 53 intake is 54
                AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                        drivetrainSubsystem,
                        arm,
                        shooter,
                        transfer,
                        intake,
                        GamePieceProcessor.getInstance(),
                        kStartPathDallas,
                        kShootParam32,
                        Rotation2d.fromDegrees(-90)),
                // Shoot 53, use pathfindToPose flipped to go to shooting position
                AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                        .deadlineWith(
                                new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                                new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)),
                // Get 52, use Pathfollow to go to 52 position, fallback for 52 intake is 51
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake,
                        GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(kShootPoseUnderStageTo52Path)),
                        Rotation2d.fromDegrees(90)),
                // Score 52, use pathfindToPose flipped to go to shooting position
                AutoBuilder.pathfindToPoseFlipped(kShootPoseNearSide, PathfindConstants.constraints)
                        .deadlineWith(
                                new SetArmAngleCommand(arm, kShootParamNearSide.angle_deg),
                                new SetShooterTargetCommand(shooter, kShootParamNearSide.speed_rps)),
                // Get 51, use Pathfollow to go to 51 position, fallback for 51 intake is 52
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake,
                        GamePieceProcessor.getInstance(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(kShootPoseNearSideTo51Path)),
                        Rotation2d.fromDegrees(-90)),
                // Score 51, use pathfindToPose flipped to go to shooting position
                AutoBuilder.pathfindToPoseFlipped(kShootPoseNearSide, PathfindConstants.constraints),
                // End, for now we just chase 54, it should be stopping at the midline
                AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake,
                        GamePieceProcessor.getInstance(),
                        AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_54_POSITION,
                                PathfindConstants.constraints),
                        Rotation2d.fromDegrees(-90)));
    }

    private Command buildWingShot53GoForFarSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'buildWingShot53GoForFarSide'");
    }

    private Command buildWingShot53GoForNearSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'buildWingShot53GoForNearSide'");
    }

    private Command buildDrop53GoForFarSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'buildDrop53GoForFarSide'");
    }

    private Command buildDrop53GoForNearSide(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'buildDrop53GoForNearSide'");
    }

}
