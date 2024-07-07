package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class CenterLineSectionBasedAuto extends SequentialCommandGroup {
    private final DrivetrainSubsystem sDrivetrainSubsystem;
    private final GamePieceProcessor sGamePieceProcessor;
    private final Intake sIntake;
    private final Transfer sTransfer;
    private final Shooter sShooter;
    private final Arm sArm;

    private Boolean isNoteAAcquired = false;
    private Boolean isNoteBAcquired = false;

    private Command buildPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

    public CenterLineSectionBasedAuto(
            DrivetrainSubsystem drivetrainSubsystem,
            GamePieceProcessor gamePieceProcessor,
            Intake intake,
            Transfer transfer,
            Shooter shooter,
            Arm arm) {
        this.sDrivetrainSubsystem = drivetrainSubsystem;
        this.sGamePieceProcessor = gamePieceProcessor;
        this.sIntake = intake;
        this.sTransfer = transfer;
        this.sShooter = shooter;
        this.sArm = arm;

        addCommands(
                // zeroing and shoot preload
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                drivetrainSubsystem.runZeroingCommand(),
                                new InstantCommand(
                                        () -> {
                                            Optional<Alliance> a = DriverStation.getAlliance();
                                            PathPlannerPath firstPath = PathPlannerPath.fromPathFile("homeTo31");
                                            if (a.isPresent() && a.get() == Alliance.Red) {
                                                firstPath = firstPath.flipPath();
                                            }
                                            drivetrainSubsystem.setPose(firstPath.getPreviewStartingHolonomicPose());
                                        })),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SetArmAngleCommand(sArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                                        new SetShooterTargetCommand(
                                                sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                                new FeedCommand(sTransfer),
                                new InstantCommand(() -> sShooter.stop()))),
                // Note A
                new SequentialCommandGroup(
                        // 1. Move from home to ObserveA only if Note A is not acquired
                        new ConditionalCommand(buildPath("homeToObserveA"), new InstantCommand(),
                                () -> !isNoteAAcquired)),

                // 2. Check if Note A is visible at ObserveA
                new ConditionalCommand(
                        // 2a. If Note A is visible, chase and intake it, 
                        new SequentialCommandGroup(
                                new ChaseNoteCommand(sDrivetrainSubsystem, sGamePieceProcessor, sIntake, sTransfer),
                                new InstantCommand(() -> isNoteAAcquired = true)),
                        // 2b. If Note A is not visible, move to ObserveB only if Note B is not acquired
                        new ConditionalCommand(
                                buildPath("observeAToObserveB"),
                                new InstantCommand(),
                                () -> !isNoteBAcquired),
                        () -> sGamePieceProcessor.getClosestGamePieceInfo().isPresent()),

                // 3. Move from the Note A area back to home, shoot the acquired Note A
                buildPath("noteAAreaToHome")
                        .alongWith(
                                new SetShooterTargetCommand(sShooter,
                                        ShootingParameters.BELOW_SPEAKER.speed_rps))
                        .andThen(new FeedCommand(sTransfer).onlyIf(() -> sTransfer.isOmronDetected()))
                        .andThen(new InstantCommand(() -> sShooter.stop())),

                // Note B
                new SequentialCommandGroup(
                        // 4. Move from home to ObserveB only if Note B is not acquired
                        new ConditionalCommand(
                                buildPath("homeToObserveB"),
                                new InstantCommand(),
                                () -> !isNoteBAcquired),
                        // 5. Check if Note B is visible at ObserveB
                        new ConditionalCommand(
                                // 5a. If Note B is visible, chase and intake it
                                new SequentialCommandGroup(
                                        new ChaseNoteCommand(sDrivetrainSubsystem, sGamePieceProcessor, sIntake, sTransfer),
                                        new InstantCommand(() -> isNoteBAcquired = true)
                                ),
                                // 5b. If Note B is not visible, move back home
                                buildPath("observeBToHome"),
                                () -> sGamePieceProcessor.getClosestGamePieceInfo().isPresent()),

                        // 7. Move from the Note B area back to home, shoot the acquired Note B
                        buildPath("noteBAreaToHome")
                                .alongWith(
                                        new SetShooterTargetCommand(sShooter,
                                                ShootingParameters.BELOW_SPEAKER.speed_rps))
                                .andThen(new FeedCommand(sTransfer).onlyIf(() -> sTransfer.isOmronDetected()))
                                .andThen(new InstantCommand(() -> sShooter.stop())))

        );

    }
}
