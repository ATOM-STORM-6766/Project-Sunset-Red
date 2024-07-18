package frc.robot.auto.modes;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    private static final Pose2d kShootPose = new Pose2d(1.71, 6.41, Rotation2d.fromDegrees(14.0));

    // upon robot pose x reach 7.0m we start chase note
    // for reference: wing line ~5.8m
    private static final double kChaseNoteDeadlineX = 5.8;

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

        Command pathFind52 = AutoBuilder.pathfindToPoseFlipped(FiendConstants.NOTE_52_POSITION,
                PathfindConstants.constraints);

        SnapToAngleCommand snapTo90_1 = new SnapToAngleCommand(drivetrainSubsystem, () -> kZeroTranslation,
                () -> Optional.of(Rotation2d.fromDegrees(-90.0)), () -> false);
        
        SnapToAngleCommand snapTo90_2 = new SnapToAngleCommand(drivetrainSubsystem, () -> kZeroTranslation,
                () -> Optional.of(Rotation2d.fromDegrees(-90.0)), () -> false);

        // build auto
        addCommands(
                prepare,

                // go to 51
                goto51
                        .deadlineWith((new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE)
                                .alongWith(new IntakeCommand(intake, transfer))))
                        .until(() -> isChaseDeadlineReached()
                                        && sGamePieceProcessor.getClosestGamePieceInfo().isPresent()),
                // get note
                new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor,
                intake, transfer, arm),

                // if no note, rotate to find note
                Commands.either(
                        // we have note: go back and shoot(below)
                        new WaitCommand(0),
                        // we dont have note: rotate and chase another note (TODO: AT OUR SIDE)
                        new SequentialCommandGroup(
                                snapTo90_1.until(() -> snapTo90_1.isAligned()),
                                new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor, intake, transfer, arm)),
                        () -> (transfer.isOmronDetected())),

                // TODO : PATH FIND ALONG WITH SHOOTER SPIN UP

                // go back 
                buildFindPathThenShoot(),

                // shoot 
                new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake, () -> kZeroTranslation).until(()->!transfer.isOmronDetected())
                        .andThen(new InstantCommand(()->shooter.stop())),
        

                // now we go 52
                pathFind52
                        .deadlineWith(new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE)
                                .alongWith(new IntakeCommand(intake, transfer))
                        )
                        .until(
                                () -> isChaseDeadlineReached()
                                        && sGamePieceProcessor.getClosestGamePieceInfo().isPresent()),

                // get 52
                new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor,
                intake, transfer, arm),

                // if no note, rotate to find note
                Commands.either(
                        // we have note: go back and shoot(below)
                        new WaitCommand(0),
                        // we dont have note: rotate and chase another note (TODO: AT OUR SIDE)
                        new SequentialCommandGroup(
                                snapTo90_2.until(() -> snapTo90_2.isAligned()),
                                new ChaseNoteStateMachineCommand(drivetrainSubsystem, sGamePieceProcessor, intake, transfer, arm)),
                        () -> (transfer.isOmronDetected())),

                
                // TODO : PATH FIND ALONG WITH SHOOTER SPIN UP

                // go back 
                buildFindPathThenShoot(),

                // shoot 
                new VisionShootCommand(shooter, arm, transfer, drivetrainSubsystem, intake, () -> kZeroTranslation)
                        .andThen(new InstantCommand(()->shooter.stop()))
        );

    }

    Command buildFindPathThenShoot(){
        return AutoBuilder.pathfindToPoseFlipped(kShootPose, PathfindConstants.constraints);
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
