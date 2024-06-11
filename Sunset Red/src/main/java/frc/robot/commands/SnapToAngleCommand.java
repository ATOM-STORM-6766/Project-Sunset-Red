package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SnapToAngleCommand extends Command {

  private final TrapezoidProfile.Constraints swerveRotateConstraints =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(300));
  private final ProfiledPIDController snapToAnglePID =
      new ProfiledPIDController(4.0, 0, 0.2, swerveRotateConstraints);

  private final DrivetrainSubsystem mDrivetrainSubsystem;
  private final Supplier<Translation2d> driveVectorSupplier;
  /*
   * Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity
   */
  private final BooleanSupplier interruptSupplier;
  private final BooleanSupplier robotCentricSupplier;
  private final Supplier<Optional<Rotation2d>> goalHeadingSupplier;
  private Optional<Rotation2d> goalHeading = Optional.empty();
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
      BooleanSupplier robotCentricSupplier,
      BooleanSupplier interruptSupplier) {
    this.setName("snapToAmp");
    mDrivetrainSubsystem = drivetrainSubsystem;
    this.driveVectorSupplier = driveVectorSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    this.goalHeadingSupplier = goalHeadingSupplier;
    this.interruptSupplier = interruptSupplier;
    addRequirements(drivetrainSubsystem); // required for default command
  }

  public SnapToAngleCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Translation2d> driveVectorSupplier,
      Supplier<Optional<Rotation2d>> goalHeadingSupplier,
      BooleanSupplier robotCentricSupplier) {
    this(
        drivetrainSubsystem,
        driveVectorSupplier,
        goalHeadingSupplier,
        robotCentricSupplier,
        () -> false);
    this.setName("driveWithRightStick");
  }

  @Override
  public void initialize() {
    snapToAnglePID.reset(mDrivetrainSubsystem.getHeading().getRadians());
    snapToAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    snapToAnglePID.setGoal(mDrivetrainSubsystem.getHeading().getRadians());
  }

  @Override
  public void execute() {
    // Running the lambda statements and getting the velocity values

    Translation2d driveVector = driveVectorSupplier.get();
    goalHeading = goalHeadingSupplier.get();
    if (goalHeading.isPresent()) {
      snapToAnglePID.setGoal(goalHeading.get().getRadians());
    }
    mDrivetrainSubsystem.drive(
        driveVector,
        snapToAnglePID.calculate(mDrivetrainSubsystem.getHeading().getRadians()),
        !robotCentricSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return interruptSupplier.getAsBoolean();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addBooleanProperty("rightStickInputPresent:", () -> goalHeading.isPresent(), null);
  }
}
