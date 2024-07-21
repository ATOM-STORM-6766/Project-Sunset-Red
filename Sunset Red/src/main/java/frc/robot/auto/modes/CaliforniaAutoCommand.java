package frc.robot.auto.modes;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathfindConstants;
import frc.robot.commands.ChaseNoteStateMachineCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.commands.SnapToAngleCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.commands.VisionShootCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class CaliforniaAutoCommand extends SequentialCommandGroup {

        // private static final Pose2d kNearStart = new Pose2d(1.28, 6.41,
        // Rotation2d.fromDegrees(30.0));
        private static final Pose2d kShootPose = new Pose2d(1.71, 6.41, Rotation2d.fromDegrees(14.0));

        // upon robot pose x reach 7.0m we start chase note
        // for reference: wing line ~5.8m
        private static final double kChaseNoteDeadlineX = 4.8;

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
                                                                        Optional<Alliance> currentAlliance = DriverStation
                                                                                        .getAlliance();
                                                                        PathPlannerPath firstPath = PathPlannerPath
                                                                                        .fromPathFile("nearTo51");
                                                                        if (currentAlliance.isPresent()
                                                                                        && currentAlliance
                                                                                                        .get() == Alliance.Red) {
                                                                                firstPath = firstPath.flipPath();
                                                                        }
                                                                        drivetrainSubsystem.setPose(firstPath
                                                                                        .getPreviewStartingHolonomicPose());
                                                                        SmartDashboard.putString("Auto Status",
                                                                                        "Finished prepare command");

                                                                })),
                                new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                                new SetArmAngleCommand(arm,
                                                                                ShootingParameters.BELOW_SPEAKER.angle_deg),
                                                                new SetShooterTargetCommand(
                                                                                shooter,
                                                                                ShootingParameters.BELOW_SPEAKER.speed_rps)),
                                                new FeedCommand(transfer),
                                                new InstantCommand(
                                                                () -> {
                                                                        shooter.stop();
                                                                })));

                Command goto51 = new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Starting goto51")),
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("California nearTo51")),
                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Finished goto51")));

                Command pathFindto52 = new SequentialCommandGroup(
                                new InstantCommand(
                                                () -> SmartDashboard.putString("Auto Status", "Starting pathFindto53")),
                                AutoBuilder.pathfindToPoseFlipped(FieldConstants.NOTE_52_POSITION,
                                                PathfindConstants.constraints, 0, 0),
                                new InstantCommand(() -> SmartDashboard.putString("Auto Status",
                                                "Finished pathFindto53")));

                Command snapTo90_1 = new TurnToHeadingCommand(drivetrainSubsystem, Rotation2d.fromDegrees(90.0))
                        .withTolerance(Math.toRadians(3.5));

                Command snapTo90_2 = new TurnToHeadingCommand(drivetrainSubsystem, Rotation2d.fromDegrees(90.0))
                        .withTolerance(Math.toRadians(3.5));

                // build auto
                addCommands(
                                new InstantCommand(
                                                () -> SmartDashboard.putString("Auto Status", "Starting auto routine")),
                                prepare,

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Going to 51")),
                                goto51
                                                .deadlineWith((new SetArmAngleCommand(arm,
                                                                ArmConstants.INTAKE_OBSERVE_ARM_ANGLE)
                                                                .alongWith(new IntakeCommand(intake, transfer))))
                                                .until(() -> {
                                                        boolean deadline = isChaseDeadlineReached();
                                                        Optional<PhotonTrackedTarget> target = sGamePieceProcessor
                                                                        .getClosestGamePieceInfo();
                                                        boolean hasTarget = target.isPresent();
                                                        SmartDashboard.putBoolean("Chase Deadline Reached", deadline);
                                                        SmartDashboard.putBoolean("Has Target", hasTarget);
                                                        return deadline && hasTarget;
                                                }),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note")),
                                new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor, intake,
                                                transfer, arm),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Checking for note")),
                                Commands.either(
                                                new WaitCommand(0),
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> SmartDashboard.putString(
                                                                                "Auto Status",
                                                                                "Rotating to find note")),
                                                                snapTo90_1,
                                                                new InstantCommand(() -> SmartDashboard.putString(
                                                                                "Auto Status",
                                                                                "Rotation Finished")),
                                                                new ChaseNoteStateMachineCommand(drivetrainSubsystem,
                                                                                sGamePieceProcessor, intake, transfer,
                                                                                arm)),
                                                () -> {
                                                        boolean hasNote = transfer.isOmronDetected();
                                                        SmartDashboard.putBoolean("Has Note", hasNote);
                                                        return hasNote;
                                                }),

                                new InstantCommand(
                                                () -> SmartDashboard.putString("Auto Status", "Finding path to shoot")),
                                buildFindPathThenShoot(),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Shooting")),
                                new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake,
                                                () -> kZeroTranslation)
                                                .until(() -> !transfer.isOmronDetected())
                                                .andThen(new InstantCommand(() -> {
                                                        shooter.stop();
                                                        SmartDashboard.putString("Auto Status", "Finished shooting");
                                                })),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Going to 53")),
                                pathFindto52
                                                .deadlineWith(new SetArmAngleCommand(arm,
                                                                ArmConstants.INTAKE_OBSERVE_ARM_ANGLE)
                                                                .alongWith(new IntakeCommand(intake, transfer)))
                                                .until(
                                                                () -> {
                                                                        boolean deadline = isChaseDeadlineReached();
                                                                        Optional<PhotonTrackedTarget> target = sGamePieceProcessor
                                                                                        .getClosestGamePieceInfo();
                                                                        boolean hasTarget = target.isPresent();
                                                                        SmartDashboard.putBoolean(
                                                                                        "Chase Deadline Reached",
                                                                                        deadline);
                                                                        SmartDashboard.putBoolean("Has Target",
                                                                                        hasTarget);
                                                                        return deadline && hasTarget;
                                                                }),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Chasing note (53)")),
                                new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor, intake,
                                                transfer, arm),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status",
                                                "Checking for note (53)")),
                                Commands.either(
                                                new WaitCommand(0),
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> SmartDashboard.putString(
                                                                                "Auto Status",
                                                                                "Rotating to find note (53)")),
                                                                snapTo90_2,
                                                                new ChaseNoteStateMachineCommand(drivetrainSubsystem,
                                                                                sGamePieceProcessor, intake, transfer,
                                                                                arm)),
                                                () -> {
                                                        boolean hasNote = transfer.isOmronDetected();
                                                        SmartDashboard.putBoolean("Has Note", hasNote);
                                                        return hasNote;
                                                }),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status",
                                                "Finding path to shoot (53)")),
                                buildFindPathThenShoot(),

                                new InstantCommand(() -> SmartDashboard.putString("Auto Status", "Shooting (53)")),
                                new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake,
                                                () -> kZeroTranslation)
                                                .andThen(new InstantCommand(() -> {
                                                        shooter.stop();
                                                        SmartDashboard.putString("Auto Status",
                                                                        "Finished auto routine");
                                                })));
        }

        Command buildFindPathThenShoot() {
                return AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints);
        }

        @Override
        public InterruptionBehavior getInterruptionBehavior() {
                return InterruptionBehavior.kCancelIncoming;
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
