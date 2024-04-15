package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SnapToAngleCommand extends Command {
  private final DrivetrainSubsystem mDrivetrainSubsystem;
  private final Supplier<Translation2d> driveVectorSupplier;
  /*
   * Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity
   */
  private final BooleanSupplier interruptSupplier;  
  private final BooleanSupplier robotCentricSupplier;
  private final Supplier<Optional<Rotation2d>> goalHeadingSupplier;
  /**
   * The default drive command constructor
   *
   * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
   * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the
   *     max velocity
   * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the
   *     max velocity
   */
  public SnapToAngleCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Optional<Rotation2d>> goalHeadingSupplier,
      BooleanSupplier interruptSupplier, 
      BooleanSupplier robotCentricSupplier) {
    mDrivetrainSubsystem = drivetrainSubsystem;
    this.driveVectorSupplier = driveVectorSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    this.goalHeadingSupplier = goalHeadingSupplier;
    this.interruptSupplier = interruptSupplier;
    addRequirements(drivetrainSubsystem); // required for default command
  }

  @Override
  public void execute() {
    // Running the lambda statements and getting the velocity values
    Optional<Rotation2d> goalHeading;
    Translation2d driveVector = driveVectorSupplier.get();
    goalHeading = goalHeadingSupplier.get();

    mDrivetrainSubsystem.drive(
        driveVector, 0.0, goalHeading, !robotCentricSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, Optional.empty(), true);
  }


    @Override
    public boolean isFinished() {
        return interruptSupplier.getAsBoolean();
    }

}
