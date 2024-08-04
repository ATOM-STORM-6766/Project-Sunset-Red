package frc.robot.auto.modes.Dallas;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.commands.NavTrapCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.TrapFan;
import frc.robot.utils.ShootingParameters;

public class DallasAutoTrap53Routine {
    // define the start path for the robot
    private static final String kStartPathDallas = "Dallas StartPath";

    // define shooting params
    private static final ShootingParameters kShootParam32 = new ShootingParameters(75, 36.5);


    /**
     * Build the command for the TRAP53 strategy Start at Dallas mid position, move to 53, shoot 32
     * along the way Then move to the nearest trap position
     * 
     * Fallback strategy for 53 intake: go for 54 (-90 degree)
     * 
     * @param drivetrainSubsystem
     * @param arm
     * @param shooter
     * @param transfer
     * @param intake
     * @param trapFan
     * @return
     */
    public static Command getTrap53Command(DrivetrainSubsystem drivetrainSubsystem, Arm arm, Shooter shooter,
            Transfer transfer, Intake intake, TrapFan trapFan, Rotation2d fallbackRotation53) {
        return new SequentialCommandGroup(
                // Move to 53, intake and shoot 32 along the way
                AutoCommandFactory.buildIntakeShootWhileMovingCommand(drivetrainSubsystem, arm,
                        shooter, transfer, intake, GamePieceProcessor.getInstance(),
                        kStartPathDallas, kShootParam32, fallbackRotation53),

                // Move to the nearest trap position
                new NavTrapCommand(drivetrainSubsystem, arm, shooter, intake, transfer, trapFan));
    }
}
