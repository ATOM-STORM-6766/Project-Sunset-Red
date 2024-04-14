package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DefaultDriveCommand extends Command {
  private final DrivetrainSubsystem mDrivetrainSubsystem;

  private final Supplier<Translation2d> driveVectorSupplier;
  /*
   * Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity
   */

  private final BooleanSupplier robotCentricSupplier;
  private final Supplier<Optional<Rotation2d>> goalHeadingSupplier;
  private final Supplier<Double> rawRotationRateSupplier;
  /**
   * The default drive command constructor
   *
   * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
   * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the
   *     max velocity
   * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the
   *     max velocity
   */
  public DefaultDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Double> rawRotationRateSupplier,
      Supplier<Optional<Rotation2d>> goalHeadingSupplier,
      BooleanSupplier robotCentricSupplier) {
    mDrivetrainSubsystem = drivetrainSubsystem;
    this.driveVectorSupplier = driveVectorSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    this.goalHeadingSupplier = goalHeadingSupplier;
    this.rawRotationRateSupplier = rawRotationRateSupplier;
    addRequirements(drivetrainSubsystem); // required for default command
  }

  @Override
  public void execute() {
    // Running the lambda statements and getting the velocity values
    double angularVelocity;
    Optional<Rotation2d> goalHeading;
    Translation2d driveVector = driveVectorSupplier.get();

    angularVelocity = rawRotationRateSupplier.get();
    goalHeading = goalHeadingSupplier.get();
    mDrivetrainSubsystem.drive(
        driveVector, angularVelocity, goalHeading, !robotCentricSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    // todo: change new Rotation2d() to currentHeading
    mDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, Optional.of(new Rotation2d()), true);
  }

  @Override
  public boolean isFinished() {
    return false; // Default command never ends
  }
}
