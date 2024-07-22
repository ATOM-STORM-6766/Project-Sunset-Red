package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.TrapFan;
import frc.robot.utils.ShootingParameters;

public class BlowTrapAndDropCommand extends SequentialCommandGroup {
  private final TrapFan sTrapFan;
  private final Shooter sShooter;
  private final Arm sArm;
  private final Transfer sTransfer;
  private final double fanSpeed;

  public BlowTrapAndDropCommand(TrapFan trapFan, Shooter shooter, Arm arm, Transfer transfer,
      double fanSpeed) {
    this.sTrapFan = trapFan;
    this.sShooter = shooter;
    this.sArm = arm;
    this.sTransfer = transfer;
    this.fanSpeed = fanSpeed;

    // Create the blow trap command
    Command blowTrapCommand = new Command() {
      @Override
      public void initialize() {
        sTrapFan.setFanSpeed(fanSpeed);
      }

      @Override
      public void end(boolean interrupted) {
        sTrapFan.stopFan();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };

    // Add logging and SmartDashboard updates
    Command logStarted = new InstantCommand(() -> {
      System.out.println("BlowTrapAndDrop: Started");
      SmartDashboard.putString("BlowTrapAndDrop/Status", "Running");
    });

    Command logFinished = new InstantCommand(() -> {
      System.out.println("BlowTrapAndDrop: Finished");
      SmartDashboard.putString("BlowTrapAndDrop/Status", "Finished");
    });

    // Add the commands to the sequence
    addCommands(logStarted,
        new SetShooterTargetCommand(sShooter, ShootingParameters.TRAP.speed_rps)
            .alongWith(new SetArmAngleCommand(sArm, ShootingParameters.TRAP.angle_deg)),
        new SequentialCommandGroup(new WaitCommand(2), // Wait for 2 seconds
            new FeedCommand(sTransfer)).deadlineWith(blowTrapCommand),
        logFinished);

    addRequirements(trapFan, shooter, arm, transfer);
  }
}
