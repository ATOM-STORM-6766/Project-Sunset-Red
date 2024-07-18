package frc.robot.auto.modes;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FiendConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.commands.ChaseNoteStateMachineCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.commands.SnapToAngleCommand;
import frc.robot.commands.VisionShootCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class CaliforniaAutoCommand extends SequentialCommandGroup {

    // private static final Pose2d kNearStart = new Pose2d(1.28, 6.41,
    // Rotation2d.fromDegrees(30.0));
    private static final Pose2d kShootPose = new Pose2d(3.71, 6.41, Rotation2d.fromDegrees(14.0));

    // upon robot pose x reach 7.0m we start chase note
    // for reference: wing line ~5.8m
    private static final double kChaseNoteDeadlineX = 7.0;

    private static final Translation2d kZeroTranslation = new Translation2d();

    private DrivetrainSubsystem sDrivetrainSubsystem;
    private GamePieceProcessor sGamePieceProcessor;

    // near 30.0 degree start, preload and go 51 (between 31 32)
    // shoot 51 and go 52
    public CaliforniaAutoCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Arm arm,
            Shooter shooter,
            Transfer transfer,
            Intake intake) {

        sDrivetrainSubsystem = drivetrainSubsystem;
        sGamePieceProcessor = GamePieceProcessor.getInstance();

        Command prepare = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        drivetrainSubsystem.runZeroingCommand(),
                        new InstantCommand( // maybe don't need, will use vision to override
                                () -> {
                                    Optional<Alliance> a = DriverStation.getAlliance();
                                    PathPlannerPath firstPath = PathPlannerPath.fromPathFile("nearTo51");
                                    if (a.isPresent() && a.get() == Alliance.Red) {
                                        firstPath = firstPath.flipPath();
                                    }
                                    drivetrainSubsystem.setPose(firstPath.getPreviewStartingHolonomicPose());
                                })),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetArmAngleCommand(arm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                                new SetShooterTargetCommand(
                                        shooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                        new FeedCommand(transfer),
                        new InstantCommand(
                                () -> {
                                    shooter.stop();
                                })));

        Command goto51 = AutoBuilder.followPath(PathPlannerPath.fromPathFile("nearTo51"));

        Command chaseNote = new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor,
                intake, transfer, arm);

        Command findPathThenShoot = AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints);

        Command staticVisionShoot = new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake,
                () -> kZeroTranslation);

        Command pathFind52 = AutoBuilder.pathfindToPoseFlipped(FiendConstants.NOTE_52_POSITION,
                PathfindConstants.constraints);

        SnapToAngleCommand snapTo90 = new SnapToAngleCommand(drivetrainSubsystem, () -> kZeroTranslation,
                () -> Optional.of(Rotation2d.fromDegrees(90.0)), () -> false);

        // build auto
        addCommands(
                prepare,

                goto51.alongWith(
                        new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                        new IntakeCommand(intake, transfer)).until(
                                () -> isChaseDeadlineReached()
                                        && sGamePieceProcessor.getClosestGamePieceInfo().isPresent()),
                chaseNote,
                Commands.either(
                        // we have note: go back and shoot(below)
                        Commands.none(),
                        // we dont have note: rotate and chase another note (TODO: AT OUR SIDE)
                        new SequentialCommandGroup(
                                snapTo90.until(() -> snapTo90.isAligned()),
                                chaseNote),
                        () -> transfer.isOmronDetected()),
                // TODO : PATH FIND ALONG WITH SHOOTER SPIN UP
                findPathThenShoot,
                staticVisionShoot,

                // now we do 52
                pathFind52.alongWith(
                        new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE),
                        new IntakeCommand(intake, transfer)).until(
                                () -> isChaseDeadlineReached()
                                        && sGamePieceProcessor.getClosestGamePieceInfo().isPresent()),
                chaseNote,
                Commands.either(
                        // we have note: go back and shoot(below)
                        Commands.none(),
                        // we dont have note: rotate and chase another note (TODO: AT OUR SIDE)
                        new SequentialCommandGroup(
                                snapTo90.until(() -> snapTo90.isAligned()),
                                chaseNote),
                        () -> transfer.isOmronDetected()),
                // TODO : PATH FIND ALONG WITH SHOOTER SPIN UP
                findPathThenShoot,
                staticVisionShoot);
    }

    private boolean isChaseDeadlineReached() {
        Optional<Alliance> a = DriverStation.getAlliance();
        double robotX = sDrivetrainSubsystem.getPose().getX();
        if (a.isPresent() && a.get() == Alliance.Red) { // red
            return robotX < 16.54 - kChaseNoteDeadlineX;
        } else { // blue
            return robotX > kChaseNoteDeadlineX;
        }
    }

}
