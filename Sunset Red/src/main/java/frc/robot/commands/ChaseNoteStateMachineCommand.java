package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ChaseNoteStateMachineCommand extends Command {
  private enum State {
    CHASING,
    INTAKING,
    END
  }

  private final DrivetrainSubsystem sDrivetrainSubsystem;
  private final GamePieceProcessor sGamePieceProcessor;
  private final Intake sIntake;
  private final Transfer sTransfer;
  private final Arm sArm;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private State currentState;

  private SetArmAngleCommand setArmAngleCommand;
  // Constructor
  public ChaseNoteStateMachineCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      GamePieceProcessor gamePieceProcessor,
      Intake intake,
      Transfer transfer,
      Arm arm) {
    this.sDrivetrainSubsystem = drivetrainSubsystem;
    this.sGamePieceProcessor = gamePieceProcessor;
    this.sIntake = intake;
    this.sTransfer = transfer;
    this.sArm = arm;
    this.setArmAngleCommand = new SetArmAngleCommand(arm, ArmConstants.INTAKE_OBSERVE_ARM_ANGLE);

    xController = new PIDController(0.2, 0.0, 0.0); // Adjust PID values as needed
    yController = new PIDController(0.0, 0.0, 0.0); // Adjust PID values as needed
    rotationController = new PIDController(0.15, 0.01, 0); // Adjust PID values as needed

    double currentTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    addRequirements(drivetrainSubsystem, gamePieceProcessor, intake, transfer, arm);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotationController.reset();
    currentState = State.CHASING;
    setArmAngleCommand.initialize();
    SmartDashboard.putString("ChaseNote State", "CHASING");
    
  }

  @Override
  public void execute() {
    setArmAngleCommand.execute();
    double currentTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    switch (currentState) {
      case CHASING:
        SmartDashboard.putString("ChaseNote State", "CHASING");
        Optional<PhotonTrackedTarget> targetOptional = sGamePieceProcessor.getClosestGamePieceInfo();
        boolean isTargetPresent = targetOptional.isPresent();

        SmartDashboard.putBoolean("Target Present", isTargetPresent);
        SmartDashboard.putBoolean("Transfer Omron", sTransfer.isOmronDetected());

        if (isTargetPresent && !sTransfer.isOmronDetected()) {
          PhotonTrackedTarget target = targetOptional.get();
          double yawMeasure = target.getYaw();
          double pitchMeasure = target.getPitch();

          SmartDashboard.putNumber("Target Yaw", yawMeasure);
          SmartDashboard.putNumber("Target Pitch", pitchMeasure);

          if (sIntake.isOmronDetected() || pitchMeasure < 3) {
            currentState = State.INTAKING;
            SmartDashboard.putString("ChaseNote State", "INTAKING");
          } else {
            // drive the robot
            double xOutput = -xController.calculate(pitchMeasure, -5);
            double yOutput = yController.calculate(yawMeasure, 0);
            double angularVelocity = rotationController.calculate(yawMeasure, 0);

            SmartDashboard.putNumber("Drive X Output", xOutput);
            SmartDashboard.putNumber("Drive Y Output", yOutput);
            SmartDashboard.putNumber("Angular Velocity", angularVelocity);

            Translation2d driveVector = new Translation2d(xOutput, yOutput);
            sDrivetrainSubsystem.drive(driveVector, angularVelocity, false);
            sIntake.setIntake();
            sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
          }
        } else {
          currentState = State.END;
          SmartDashboard.putString("ChaseNote State", "END (No Target)");
        }
        break;

      case INTAKING:
        SmartDashboard.putString("ChaseNote State", "INTAKING");
        sIntake.setIntake();
        sTransfer.setVoltage(Transfer.INTAKE_VOLTS);
        sDrivetrainSubsystem.drive(new Translation2d(1.5, 0), 0, false);
        SmartDashboard.putBoolean("Transfer Omron", sTransfer.isOmronDetected());
        if (sTransfer.isOmronDetected()) {
          currentState = State.END;
          SmartDashboard.putString("ChaseNote State", "END (Note Acquired)");
        }
        break;

      case END:
        SmartDashboard.putString("ChaseNote State", "END");
        sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    sIntake.stop();
    sTransfer.stop();
    setArmAngleCommand.end(interrupted);
    SmartDashboard.putString("ChaseNote State", "ENDED");
    SmartDashboard.putBoolean("ChaseNote Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return currentState == State.END;
  }
}