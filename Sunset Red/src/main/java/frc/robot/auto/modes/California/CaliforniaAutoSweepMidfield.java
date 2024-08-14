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

public class CaliforniaAutoSweepMidfield {

    public static Command buildSweepMidFieldCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Arm arm,
            Shooter shooter,
            Transfer transfer,
            Intake intake) {

        return new SequentialCommandGroup(
            // Prepare routine
            AutoCommandFactory.buildPrepCommand(
                drivetrainSubsystem,
                ShootingParameters.BELOW_SPEAKER,
                AutoRoutineConfig.AutoPaths.START_CALIFORNIA,
                arm,
                shooter,
                transfer),

            // Move to 51, intake and shoot
            AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoRoutineConfig.AutoPaths.START_CALIFORNIA,
                AutoRoutineConfig.AutoShootPositions.NOTE_31.shootParams,
                Rotation2d.fromDegrees(-90)),

            // Go to near side shoot pose and score first note
            AutoBuilder.pathfindToPoseFlipped(AutoRoutineConfig.AutoShootPositions.NEAR_SIDE.shootPose, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, AutoRoutineConfig.AutoShootPositions.NEAR_SIDE.shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, AutoRoutineConfig.AutoShootPositions.NEAR_SIDE.shootParams.speed_rps)),

            // Move to 52 and intake
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(AutoRoutineConfig.AutoPaths.NEAR_SIDE_TO_52)),
                Rotation2d.fromDegrees(-90)),

            // Go to near side shoot pose and score second note
            AutoBuilder.pathfindToPoseFlipped(AutoRoutineConfig.AutoShootPositions.NEAR_SIDE.shootPose, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, AutoRoutineConfig.AutoShootPositions.NEAR_SIDE.shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, AutoRoutineConfig.AutoShootPositions.NEAR_SIDE.shootParams.speed_rps)),

            // Move to 53 and intake
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(AutoRoutineConfig.AutoPaths.NEAR_SIDE_TO_53)),
                Rotation2d.fromDegrees(-90)),

            // Go to under stage shoot pose and score third note
            AutoBuilder.pathfindToPoseFlipped(AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootPose, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.angle_deg),
                    new SetShooterTargetCommand(shooter, AutoRoutineConfig.AutoShootPositions.UNDER_STAGE.shootParams.speed_rps)),

            // Move to 54 and intake
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem,
                arm,
                shooter,
                transfer,
                intake,
                GamePieceProcessor.getInstance(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(AutoRoutineConfig.AutoPaths.UNDER_STAGE_TO_54)),
                Rotation2d.fromDegrees(-90)),

            // Finally move to 55 (with 54 inside the robot)
            // just follow the path 54 to 55, without chasing any note
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(AutoRoutineConfig.AutoPaths.FROM_54_TO_55))

        );
    }
}