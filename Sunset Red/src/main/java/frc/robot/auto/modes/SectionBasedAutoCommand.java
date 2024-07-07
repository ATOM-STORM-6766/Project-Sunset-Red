package frc.robot.auto.modes;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ChaseNoteCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class SectionBasedAutoCommand extends SequentialCommandGroup {
    private Intake sIntake;
    private Shooter sShooter;
    private Arm sArm;
    private Transfer sTransfer;
    private DrivetrainSubsystem sDrivetrainSubsystem;
    public SectionBasedAutoCommand(
            Intake intake,
            Shooter shooter,
            Arm arm,
            Transfer transfer,
            DrivetrainSubsystem drivetrainSubsystem) {
        sIntake = intake;
        sShooter = shooter;
        sArm = arm;
        sTransfer = transfer;
        sDrivetrainSubsystem = drivetrainSubsystem;
    

        addCommands(
                // zeroing and shoot preload
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                sDrivetrainSubsystem.runZeroingCommand(),
                                new InstantCommand(
                                        () -> {
                                            Optional<Alliance> a = DriverStation.getAlliance();
                                            PathPlannerPath firstPath = PathPlannerPath.fromPathFile("HomeA to ObsA");
                                            if (a.isPresent() && a.get() == Alliance.Red) {
                                                firstPath = firstPath.flipPath();
                                            }
                                            sDrivetrainSubsystem.setPose(firstPath.getPreviewStartingHolonomicPose());
                                        })),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SetArmAngleCommand(sArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                                        new SetShooterTargetCommand(
                                                sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                                new FeedCommand(sTransfer),
                                new InstantCommand(
                                        () -> {
                                            sShooter.stop();
                                        }),
                                new SetArmAngleCommand(arm, Constants.ArmConstants.ARM_REST_ANGLE))),

                // 1. Go from HomeA to ObsA
                buildPath("HomeA to ObsA").deadlineWith(new IntakeCommand(sIntake, sTransfer)),

                // 2. Start chase
                new ChaseNoteCommand(sDrivetrainSubsystem, GamePieceProcessor.getInstance(), sIntake, sTransfer)
                        .withTimeout(1),

                // 3 & 4. If chase successful, go back and shoot. If not, go to ObsB
                new ConditionalCommand(
                        // If chase successful
                        buildPathAndShoot("ReturnA to HomeA"),
                        // If chase not successful
                        new SequentialCommandGroup(
                                buildPath("ObsA to ObsB"),
                                // 5. Start chase again
                                new ChaseNoteCommand(sDrivetrainSubsystem, GamePieceProcessor.getInstance(), sIntake,
                                        sTransfer)
                                        .withTimeout(1)),
                        () -> sIntake.isOmronDetected()),

                // 6. No matter the outcome of the second chase, go back and shoot
                buildPathAndShoot("ReturnB to HomeB"));
    }

    private Command buildPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

    private Command buildPathAndShoot(String pathName) {
        return new SequentialCommandGroup(
                buildPath(pathName),
                new ParallelCommandGroup(
                        new SetArmAngleCommand(sArm, ShootingParameters.BELOW_SPEAKER.angle_deg),
                        new SetShooterTargetCommand(sShooter, ShootingParameters.BELOW_SPEAKER.speed_rps)),
                new FeedCommand(sTransfer),
                new InstantCommand(() -> sShooter.stop()),
                new SetArmAngleCommand(sArm, Constants.ArmConstants.ARM_REST_ANGLE));
    }
}