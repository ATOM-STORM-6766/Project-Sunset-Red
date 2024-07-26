package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class DallasAuto extends SequentialCommandGroup {

    private static final String kStartPathName53 = "Dallas midTo53";
    private static final String kStartPathName52 = "Dallas midTo52";
    private static final ShootingParameters kShootParamUnderStage = new ShootingParameters(75, 32);
    private static final ShootingParameters kShootParam32 = new ShootingParameters(75, 45);
    private static final Pose2d kShootPoseUnderStage = new Pose2d(4.5, 4.34, Rotation2d.fromDegrees(-14));

    public enum DallasStrategy {
        START_53,
        START_52,
        START_53_THEN_TRAP
    }

    public DallasAuto(
            DrivetrainSubsystem drivetrainSubsystem,
            Arm arm,
            Shooter shooter,
            Transfer transfer,
            Intake intake,
            TrapFan trapFan,
            DallasStrategy strategy) {

        addCommands(
            // Prepare and shoot preload
            AutoCommandFactory.buildPrepCommand(
                drivetrainSubsystem,
                ShootingParameters.BELOW_SPEAKER,
                strategy == DallasStrategy.START_52 ? kStartPathName52 : kStartPathName53,
                arm,
                shooter,
                transfer
            )
        );

        switch (strategy) {
            case START_53:
                addCommands(buildStart53Sequence(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case START_52:
                addCommands(buildStart52Sequence(drivetrainSubsystem, arm, shooter, transfer, intake));
                break;
            case START_53_THEN_TRAP:
                addCommands(buildStart53ThenTrapSequence(drivetrainSubsystem, arm, shooter, transfer, intake, trapFan));
                break;
        }
    }

    private Command buildStart53Sequence(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter, Transfer transfer, Intake intake) {
        return new SequentialCommandGroup(
            // Move to 53, intaking and shooting 32 on the way
            AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake,GamePieceProcessor.getInstance(),
                kStartPathName53, kShootParam32, Rotation2d.fromDegrees(90)
            ),
            // Shoot 53
            AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                    new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)
                ),
            // Get and shoot 54
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake,GamePieceProcessor.getInstance(),
                AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_54_POSITION, PathfindConstants.constraints),
                Rotation2d.fromDegrees(90)
            ),
            AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                    new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)
                ),
            // Get 55
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake,GamePieceProcessor.getInstance(),
                AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_55_POSITION, PathfindConstants.constraints),
                Rotation2d.fromDegrees(0)
            )
        );
    }

    private Command buildStart52Sequence(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter, Transfer transfer, Intake intake) {
        return new SequentialCommandGroup(
            // Move to 52, intaking and shooting 32 on the way
            AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(),
                kStartPathName52, kShootParam32, Rotation2d.fromDegrees(90)
            ),
            // Shoot 52
            AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                    new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)
                ),
            // Get and shoot 53
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(),
                AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_53_POSITION, PathfindConstants.constraints),
                Rotation2d.fromDegrees(90)
            ),
            AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                    new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)
                ),
            // Get 54
            AutoCommandFactory.buildPathThenChaseNoteCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(),
                AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_54_POSITION, PathfindConstants.constraints),
                Rotation2d.fromDegrees(0)
            )
        );
    }

    private Command buildStart53ThenTrapSequence(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter, Transfer transfer, Intake intake, TrapFan trapFan) {
        return new SequentialCommandGroup(
            // Move to 53, intaking and shooting 32 on the way
            AutoCommandFactory.buildIntakeShootWhileMovingCommand(
                drivetrainSubsystem, arm, shooter, transfer, intake, GamePieceProcessor.getInstance(),
                kStartPathName53, kShootParam32, Rotation2d.fromDegrees(90)
            ),
            // Shoot 53
            AutoBuilder.pathfindToPoseFlipped(kShootPoseUnderStage, PathfindConstants.constraints)
                .deadlineWith(
                    new SetArmAngleCommand(arm, kShootParamUnderStage.angle_deg),
                    new SetShooterTargetCommand(shooter, kShootParamUnderStage.speed_rps)
                ),
            // Go to trap and shoot
            new NavTrapCommand(drivetrainSubsystem, arm, shooter, intake, transfer, trapFan)
        );
    }
}