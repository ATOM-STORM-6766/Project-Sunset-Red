package frc.robot.commands;

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
  private final Supplier<Optional<Double>> angularVelocitySupplier;

  private final BooleanSupplier robotCentricSupplier;

  /**
   * The default drive command constructor
   *
   * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
   * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the
   *     max velocity
   * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the
   *     max velocity
   * @param angularVelocitySupplier Gets the joystick value for the angular velocity and multiplies
   *     it by the max angular velocity
   */
  public DefaultDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Optional<Double>> angularVelocitySupplier,
      BooleanSupplier robotCentricSupplier) {
    mDrivetrainSubsystem = drivetrainSubsystem;
    this.driveVectorSupplier = driveVectorSupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    addRequirements(drivetrainSubsystem); // required for default command
  }

  @Override
  public void execute() {
    // Running the lambda statements and getting the velocity values
    double angularVelocity;

    Translation2d driveVector = driveVectorSupplier.get();
    angularVelocity = angularVelocitySupplier.get().orElse(0.0);
    mDrivetrainSubsystem.drive(driveVector, angularVelocity, !robotCentricSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return false; // Default command never ends
  }
}
