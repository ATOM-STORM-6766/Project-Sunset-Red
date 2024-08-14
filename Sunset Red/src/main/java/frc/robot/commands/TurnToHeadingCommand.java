package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToHeadingCommand extends Command {

  private final DrivetrainSubsystem sDrivetrainSubsystem;

  private Rotation2d mTargetHeading;
  private double mTolerance = Math.toRadians(1.0); // 1 deg of tolerance
  // input in radians and output in rad/s
  private final ProfiledPIDController mProfiledPID =
      new ProfiledPIDController(
          3.5,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Math.toRadians(540), Math.toRadians(720)),
          Constants.kPeriodicDt);

  private static final Translation2d kZeroTranslation = new Translation2d();

  // turn to given heading once and finished when target reached
  public TurnToHeadingCommand(DrivetrainSubsystem drivetrainSubsystem, Rotation2d targetHeading) {
    sDrivetrainSubsystem = drivetrainSubsystem;
    mTargetHeading = targetHeading;
    addRequirements(sDrivetrainSubsystem);
  }

  public TurnToHeadingCommand withPID(double kP, double kI, double kD) {
    mProfiledPID.setPID(kP, kI, kD);
    return this;
  }

  public TurnToHeadingCommand withTolerance(double tolerance_rad) {
    mTolerance = tolerance_rad;
    return this;
  }

  @Override
  public void initialize() {
    mProfiledPID.reset(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
    if (mTargetHeading != null) {
      mProfiledPID.setGoal(mTargetHeading.getRadians());
    }
    mProfiledPID.setTolerance(mTolerance);
  }

  @Override
  public void execute() {
    // in rad/s
    double turnspeed =
        mProfiledPID.calculate(sDrivetrainSubsystem.getHeading().getRadians())
            + mProfiledPID.getSetpoint().velocity;

    SmartDashboard.putString(
        "Turn to: string",
        String.format(
            "poserr: %.5f, setpoint vel: %.5f, turnspeed: %.5f, drive heading rad: %.5f",
            mProfiledPID.getPositionError(),
            mProfiledPID.getSetpoint().velocity,
            turnspeed,
            sDrivetrainSubsystem.getHeading().getRadians()));
    sDrivetrainSubsystem.drive(kZeroTranslation, turnspeed, true);
  }

  @Override
  public boolean isFinished() {
    return mTargetHeading == null || mProfiledPID.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
  }
}
