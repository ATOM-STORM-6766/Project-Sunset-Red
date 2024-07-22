package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class NavTrapCommand extends SequentialCommandGroup {

  public NavTrapCommand(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
      Intake intake, Transfer transfer, TrapFan trapFan) {

    DriveToNearestTrapCommand driveToNearestTrap =
        new DriveToNearestTrapCommand(drivetrainSubsystem);

    // TODO : NO NOTE CHECK
    addCommands(
        new ParallelCommandGroup(driveToNearestTrap,
            new SetArmAngleCommand(arm, ShootingParameters.TRAP.angle_deg),
            new SetShooterTargetCommand(shooter, ShootingParameters.TRAP.speed_rps)),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(new WaitCommand(2.0), new FeedCommand(transfer)),
            trapFan.blowTrapCommand()),
        new ParallelCommandGroup(new SetArmAngleCommand(arm, ArmConstants.ARM_REST_ANGLE),
            Commands.runOnce(() -> shooter.stop(), shooter)));
  }
}
