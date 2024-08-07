package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriveWithFollowHeadingCommand extends Command {

  private final DrivetrainSubsystem sDrivetrainSubsystem;
  // move speed in m/s
  private Supplier<Translation2d> mDriveVectorSupplier;
  private Supplier<Optional<Rotation2d>> mTargetHeadingSupplier;
  private BooleanSupplier mRobotCentricSupplier;

  // input in radians and output in rad/s
  // TODO : CHECK PID
  private final ProfiledPIDController mProfiledPID =
      new ProfiledPIDController(
          0.5, 0, 1.0, new TrapezoidProfile.Constraints(Math.toRadians(540), Math.toRadians(720)));

  // accept drive supplier and target heading
  // use a profiled controller to do the heading following
  // NOTE: this command dont finish itself.
  public DriveWithFollowHeadingCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Optional<Rotation2d>> targetHeadingSupplier,
      BooleanSupplier robotCentricSupplier) {

    sDrivetrainSubsystem = drivetrainSubsystem;
    mDriveVectorSupplier = driveVectorSupplier;
    mTargetHeadingSupplier = targetHeadingSupplier;
    mRobotCentricSupplier = robotCentricSupplier;
    addRequirements(sDrivetrainSubsystem);
  }

  public DriveWithFollowHeadingCommand withPID(double kP, double kI, double kD) {
    mProfiledPID.setPID(kP, kI, kD);
    return this;
  }

  @Override
  public void initialize() {
    mProfiledPID.reset(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
    mProfiledPID.setGoal(sDrivetrainSubsystem.getHeading().getRadians());
    mProfiledPID.setTolerance(Math.toRadians(1.0));
  }

  @Override
  public void execute() {
    Translation2d driveVector = mDriveVectorSupplier.get();
    Optional<Rotation2d> target = mTargetHeadingSupplier.get();
    if (target.isPresent()) {
      mProfiledPID.setGoal(target.get().getRadians());
    }
    // in rad/s
    double turnspeed =
        mProfiledPID.calculate(sDrivetrainSubsystem.getHeading().getRadians())
            + mProfiledPID.getSetpoint().velocity;
    if (mProfiledPID.atGoal()) {
      turnspeed = 0.0;
    }
    sDrivetrainSubsystem.drive(
        driveVector,
        turnspeed, // output is in radians per second
        !mRobotCentricSupplier.getAsBoolean());
  }

  public boolean headingAligned() {
    return mProfiledPID.getGoal().position - sDrivetrainSubsystem.getHeading().getRadians()
        < Math.toRadians(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
  }

  // whatever you use this command as default or as coroutine
  // we just dont finish ourself and thats what it does.
  @Override
  public boolean isFinished() {
    return false;
  }
}
