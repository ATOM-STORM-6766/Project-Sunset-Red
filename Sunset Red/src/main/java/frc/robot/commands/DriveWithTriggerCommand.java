package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriveWithTriggerCommand extends Command {
  private final DrivetrainSubsystem mDrivetrainSubsystem;

  private final Supplier<Translation2d> driveVectorSupplier;
  /*
   * Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity
   */

  private final BooleanSupplier robotCentricSupplier;
  private final Supplier<Double> rawRotationRateSupplier;
  private double angularVelocity;
  /**
   * The default drive command constructor
   *
   * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
   * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the
   *     max velocity
   * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the
   *     max velocity
   */
  public DriveWithTriggerCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Double> rawRotationRateSupplier,
      BooleanSupplier robotCentricSupplier) {
    mDrivetrainSubsystem = drivetrainSubsystem;
    this.driveVectorSupplier = driveVectorSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    this.rawRotationRateSupplier = rawRotationRateSupplier;
    addRequirements(drivetrainSubsystem); // required for default command
  }

  @Override
  public void execute() {
    // Running the lambda statements and getting the velocity values
    angularVelocity =
        rawRotationRateSupplier.get()
            * DriveConstants
                .kTeleDriveMaxAngularSpeedRadiansPerSecond; // -1~1 to radians per second
    Translation2d driveVector =
        driveVectorSupplier
            .get()
            .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond); // -1~1 to meters per second
    mDrivetrainSubsystem.drive(driveVector, angularVelocity, !robotCentricSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return angularVelocity == 0.0;
  }
}
