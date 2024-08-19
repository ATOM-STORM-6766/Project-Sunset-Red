package frc.robot.auto.modes.California;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathfindConstants;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.auto.AutoRoutineConfig;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class CaliforniaAuto {

    public enum CaliforniaStrategy {
        DEFAULT
        // Add more strategies here if needed
    }

    private static class StrategyParams {
        final PathPlannerPath firstNotePath; // usually this is the startPath
        final AutoRoutineConfig.AutoShootingConfig firstNoteShootConfig; 
        final PathPlannerPath secondNotePath; // from previous shoot pos to next note
        final AutoRoutineConfig.AutoShootingConfig secondNoteShootConfig;
        final PathPlannerPath thirdNotePath;
        final AutoRoutineConfig.AutoShootingConfig thirdNoteShootConfig;
        final PathPlannerPath endPath;

        StrategyParams(
            PathPlannerPath firstNotePath,
            AutoRoutineConfig.AutoShootingConfig firstNoteShootConfig,
            PathPlannerPath secondNotePath,
            AutoRoutineConfig.AutoShootingConfig secondNoteShootConfig,
            PathPlannerPath thirdNotePath,
            AutoRoutineConfig.AutoShootingConfig thirdNoteShootConfig,
            PathPlannerPath endPath) {
            this.firstNotePath = firstNotePath;
            this.firstNoteShootConfig = firstNoteShootConfig;
            this.secondNotePath = secondNotePath;
            this.secondNoteShootConfig = secondNoteShootConfig;
            this.thirdNotePath = thirdNotePath;
            this.thirdNoteShootConfig = thirdNoteShootConfig;
            this.endPath = endPath;
        }
    }

    private static StrategyParams getStrategyParams(CaliforniaStrategy strategy) {
        switch (strategy) {
            case DEFAULT:
                return new StrategyParams(
                    AutoRoutineConfig.AutoPaths.START_CALIFORNIA,
                    AutoRoutineConfig.AutoShootPositions.NEAR_SIDE,
                    AutoRoutineConfig.AutoPaths.NEAR_SIDE_TO_52,
                    AutoRoutineConfig.AutoShootPositions.UNDER_STAGE,
                    AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_53,
                    AutoRoutineConfig.AutoShootPositions.UNDER_STAGE,
                    AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_54);
            default:
                throw new IllegalArgumentException("Invalid strategy for CaliforniaAutoScore515253");
        }
    }

    public static Command buildCaliforniaAuto(
        DrivetrainSubsystem drivetrainSubsystem,
        Arm arm,
        Shooter shooter,
        Transfer transfer,
        Intake intake,
        CaliforniaStrategy strategy) {

        StrategyParams params = getStrategyParams(strategy);

        return new SequentialCommandGroup(
            // Prepare routine
            AutoCommandFactory.buildPrepCommand(
                drivetrainSubsystem,
                ShootingParameters.BELOW_SPEAKER,
                params.firstNotePath,
                arm,
                shooter,
                transfer),

            // Move to first note, intake and shoot
            AutoCommandFactory.buildPathThenChaseNoteCommand(drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(), AutoBuilder.followPath(params.firstNotePath), Rotation2d.fromDegrees(-90)),

            // Move to shoot pose and score first note
            AutoBuilder.pathfindThenFollowPath(params.firstNoteShootConfig.approachShootPosePath, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, params.firstNoteShootConfig.shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, params.firstNoteShootConfig.shootParams.speed_rps)),

            // Move to second note and intake
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoBuilder.followPath(params.secondNotePath),
                Rotation2d.fromDegrees(-90)),

            // Go to shoot pose and score second note
            AutoBuilder.pathfindThenFollowPath(params.secondNoteShootConfig.approachShootPosePath, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, params.secondNoteShootConfig.shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, params.secondNoteShootConfig.shootParams.speed_rps)),

            // Move to third note and intake
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoBuilder.followPath(params.thirdNotePath),
                Rotation2d.fromDegrees(-90)),

            // Go to shoot pose and score third note
            AutoBuilder.pathfindThenFollowPath(params.thirdNoteShootConfig.approachShootPosePath, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, params.thirdNoteShootConfig.shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, params.thirdNoteShootConfig.shootParams.speed_rps)),

            // Move to end path (if applicable)
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoBuilder.followPath(params.endPath),
                Rotation2d.fromDegrees(-90))
        );
    }
}