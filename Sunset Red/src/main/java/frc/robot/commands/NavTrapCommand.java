package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;

public class NavTrapCommand extends SequentialCommandGroup {

  public NavTrapCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Arm arm,
      Shooter shooter,
      Intake intake,
      Transfer transfer,
      TrapFan trapFan) {

    DriveToTrapCommand driveToTrap = new DriveToTrapCommand(drivetrainSubsystem);
    DriveToNearestTrapCommand driveToNearestTrap =
        new DriveToNearestTrapCommand(drivetrainSubsystem);

    addCommands(
        new ParallelCommandGroup(
            driveToNearestTrap,
            new SetArmAngleCommand(arm, ShootingParameters.TRAP.angle_deg),
            new SetShooterTargetCommand(shooter, ShootingParameters.TRAP.speed_rps)),
        Commands.either(
            new SequentialCommandGroup(
                trapFan.blowTrapCommand().withTimeout(2.0), new FeedCommand(transfer)),
            new InstantCommand(() -> shooter.stop())
                .andThen(new SetArmAngleCommand(arm, ArmConstants.ARM_REST_ANGLE)),
            () -> !driveToTrap.getLastNoTag()));
  }
}
