package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class ChaseNoteCommand extends Command {
    private final DrivetrainSubsystem sDrivetrainSubsystem;
    private final GamePieceProcessor sGamePieceProcessor;
    private final Intake sIntake;
    private final Transfer sTransfer;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private double lowestSeenPitch = Integer.MAX_VALUE;

    public ChaseNoteCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            GamePieceProcessor gamePieceProcessor,
            Intake intake,
            Transfer transfer) {
        this.sDrivetrainSubsystem = drivetrainSubsystem;
        this.sGamePieceProcessor = gamePieceProcessor;
        this.sIntake = intake;
        this.sTransfer = transfer;

        xController = new PIDController(0.2, 0.0, 0.0); // Adjust PID values as needed
        yController = new PIDController(0.0, 0.0, 0.0); // Adjust PID values as needed
        rotationController = new PIDController(0.2, 0.0, 0.0); // Adjust PID values as needed

        addRequirements(drivetrainSubsystem, gamePieceProcessor, intake, transfer);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotationController.reset();
        lowestSeenPitch = Integer.MAX_VALUE;
    }

    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> targetOptional = sGamePieceProcessor.getClosestGamePieceInfo();
        boolean isTargetPresent = targetOptional.isPresent();

        if (isTargetPresent) {
            PhotonTrackedTarget target = targetOptional.get();
            lowestSeenPitch = Math.min(lowestSeenPitch, target.getPitch());
            double yawMeasure = target.getYaw();
            double pitchMeasure = target.getPitch();

            double xOutput = -xController.calculate(pitchMeasure, -30);
            double yOutput = yController.calculate(yawMeasure, 0);

            Translation2d driveVector = new Translation2d(xOutput, yOutput);

            // Calculate rotation based on game piece position
            double angularVelocity = rotationController.calculate(yawMeasure, 0);

            sDrivetrainSubsystem.drive(driveVector, angularVelocity, false);
            sIntake.setIntake();
            sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
        } else {
            /*
             * Target not present, two cases
             * 1. target is acquired by competitor, we should cancel command immediately
             * 2. target is too close to the robot, we should continue to drive some time,
             * so that the intake omron can detect the note
             */
            if (lowestSeenPitch < -15 || sIntake.isOmronDetected()) {
                sIntake.setIntake();
                sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
                sDrivetrainSubsystem.drive(new Translation2d(1, 0), 0, false);
            } else {
                // Stop after the time limit
                sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        sIntake.stop();
        sTransfer.stop();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public boolean isFinished() {
        return sTransfer.isOmronDetected();
    }

}
