// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.auto.modes.TestAutoCommand;
import frc.robot.commands.DriveWithTriggerCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SetShooterTargetCommand;
import frc.robot.commands.SnapToAngleCommand;
import frc.robot.lib6907.CommandSwerveController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coprocessor;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SendableChooser<Command> mChooser = new SendableChooser<>();

  // * Controllers */
  private final CommandSwerveController driverController = new CommandSwerveController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  /* Subsystems */
  private final DrivetrainSubsystem sDrivetrainSubsystem = new DrivetrainSubsystem();
  private final Intake mIntake = new Intake();
  private final Transfer mTransfer = new Transfer();
  private final Shooter mShooter = new Shooter();
  private final Arm mArm = new Arm();
  private final Coprocessor mCoprocessor = Coprocessor.getInstance();

  /* pre-constructed commands */
  private final Command mZeroingCommand = sDrivetrainSubsystem.runZeroingCommand();

  private final SnapToAngleCommand mDriveWithRightStick =
      new SnapToAngleCommand(
          sDrivetrainSubsystem,
          () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
          () -> driverController.getDriveRotationAngle(), // amp heading
          () -> driverController.robotCentric());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    sDrivetrainSubsystem.setDefaultCommand(mDriveWithRightStick);

    configureBindings();
    sDrivetrainSubsystem.configureAutoBuilder();
    pushChooser();
    SmartDashboard.putData(sDrivetrainSubsystem);
    SmartDashboard.putData(mIntake);
    SmartDashboard.putData(mTransfer);
    SmartDashboard.putData(mArm);
    SmartDashboard.putData(mShooter);
    SmartDashboard.putData(mDriveWithRightStick);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /*
     * // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
     * new Trigger(m_exampleSubsystem::exampleCondition)
     * .onTrue(new ExampleCommand(m_exampleSubsystem));
     *
     * // Schedule `exampleMethodCommand` when the Xbox controller's B button is
     * // pressed,
     * // cancelling on release.
     * m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
     */

    Command resetHeadingCommand =
        new InstantCommand(
            () -> {
              sDrivetrainSubsystem.zeroHeading();
              driverController.setTranslationDirection(true);
            });
    resetHeadingCommand.addRequirements(sDrivetrainSubsystem);
    driverController.start().onTrue(resetHeadingCommand);

    new Trigger(
            () -> driverController.snapToAmpAngle() && driverController.getRawRotationRate() == 0.0)
        .onTrue(
            new SnapToAngleCommand(
                sDrivetrainSubsystem,
                () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
                () -> Optional.of(Rotation2d.fromDegrees(90.0)), // amp heading
                () -> driverController.robotCentric(),
                () -> driverController.getDriveRotationAngle().isPresent()));

    new Trigger(() -> driverController.getRawRotationRate() != 0.0)
        .onTrue(
            new DriveWithTriggerCommand(
                sDrivetrainSubsystem,
                () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
                () -> driverController.getRawRotationRate(), // amp heading
                () -> driverController.robotCentric()));

    // intake system bindings
    driverController.y().whileTrue(new IntakeCommand(mIntake, mTransfer));
    driverController.back().whileTrue(new OuttakeCommand(mIntake, mTransfer));
    // operatorController.b().whileTrue(new OuttakeCommand(mIntake, mTransfer));

    // operatorController.povLeft().onTrue(new SetArmAngleCommand(mArm, 22.5));
    // operatorController.x().onTrue(new InitializeArmCommand(mArm));
    // operatorController.y().onTrue(new SetArmAngleCommand(mArm, 50));

    // Below Speaker
    driverController
        .x()
        .whileTrue( // use whileTrue because need to cancel feed if button released
            new SetShooterTargetCommand(
                    mShooter,
                    ShootingParameters.BELOW_SPEAKER
                        .speed_rps) // this command finished means shooter reached target velocity
                .alongWith(
                    new SetArmAngleCommand(
                        mArm,
                        ShootingParameters.BELOW_SPEAKER
                            .angle_deg)) // this command finished means arm reached target angle
                .andThen(new FeedCommand(mTransfer))); // feed

    driverController
        .x()
        .onFalse(
            new InstantCommand(
                    () -> { // stop shooting, arm back
                      mShooter.stop();
                    })
                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_REST_ANGLE)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return mChooser.getSelected();
  }

  public void checkDrivetrainZeroing() {
    mZeroingCommand.schedule();
  }

  public void pushChooser() {
    // init points
    mChooser = new SendableChooser<>();

    // tested
    mChooser.setDefaultOption("example", new TestAutoCommand(sDrivetrainSubsystem));

    SmartDashboard.putData("AUTO CHOICES", mChooser);
  }
}
