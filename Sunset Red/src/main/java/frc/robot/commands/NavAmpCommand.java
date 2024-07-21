package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class NavAmpCommand extends SequentialCommandGroup {

    public NavAmpCommand(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter, Transfer transfer) {

        DriveToAmpCommand driveToAmp = new DriveToAmpCommand(drivetrainSubsystem);

        addCommands(
            new ParallelCommandGroup(
                driveToAmp,
                new SetArmAngleCommand(arm, ShootingParameters.AMP_INTERMEDIATE_POS.angle_deg),
                new SetShooterTargetCommand(shooter, ShootingParameters.AMP_LOWSPEED.speed_rps)
            ),
            new ParallelCommandGroup(
                new SetArmAngleCommand(arm, ShootingParameters.AMP_LOWSPEED.angle_deg),
                new FeedCommand(transfer)
            ),
            new InstantCommand(() -> shooter.stop()),
            new SetArmAngleCommand(arm, ArmConstants.ARM_REST_ANGLE)
        );
    }
}