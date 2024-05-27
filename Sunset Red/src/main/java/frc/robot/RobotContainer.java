// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.InitializeArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.SetArmAngleCommand;
import frc.robot.commands.SnapToAngleCommand;
import frc.robot.lib6907.CommandSwerveController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // * Controllers */
  private final CommandSwerveController driverController = new CommandSwerveController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  /* Subsystems */
  private final DrivetrainSubsystem sDrivetrainSubsystem = new DrivetrainSubsystem();
  private final Intake mIntake = new Intake();
  private final Transfer mTransfer = new Transfer();
  private final Arm mArm = new Arm();

  /* pre-constructed commands */
  private final Command mZeroingCommand = sDrivetrainSubsystem.runZeroingCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    sDrivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            sDrivetrainSubsystem,
            () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
            () -> driverController.getRawRotationRate(),
            () -> driverController.getDriveRotationAngle(),
            () -> driverController.isSlowMode()));

    configureBindings();

    SmartDashboard.putData(sDrivetrainSubsystem);
    SmartDashboard.putData(mIntake);
    SmartDashboard.putData(mTransfer);
    SmartDashboard.putData(mArm);
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
    driverController
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  sDrivetrainSubsystem.zeroHeading();
                  driverController.setTranslationDirection(true);
                }));
    new Trigger(() -> driverController.snapToAmpAngle())
        .onTrue(
            new SnapToAngleCommand(
                sDrivetrainSubsystem,
                () -> driverController.getDriveTranslation(driverController.isRobotRelative()),
                () -> Optional.of(new Rotation2d(90.0)), // amp heading
                () ->
                    driverController.getRawRotationRate() != 0.0
                        | driverController.getDriveRotationAngle().isPresent(),
                () -> driverController.isSlowMode()));

    // intake system bindings
    operatorController.a().whileTrue(new IntakeCommand(mIntake, mTransfer));
    operatorController.b().whileTrue(new OuttakeCommand(mIntake, mTransfer));

    operatorController.povLeft().onTrue(new SetArmAngleCommand(mArm, 22.5));
    operatorController.x().onTrue(new InitializeArmCommand(mArm));
    operatorController.y().onTrue(new SetArmAngleCommand(mArm, 50));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }

  public void checkDrivetrainZeroing() {
    mZeroingCommand.schedule();
  }
}
