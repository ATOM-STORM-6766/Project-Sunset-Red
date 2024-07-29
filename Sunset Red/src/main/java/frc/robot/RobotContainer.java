// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.auto.modes.*;
import frc.robot.commands.*;
import frc.robot.commands.PepGuardiolaCommand.GoalZone;
import frc.robot.lib6907.CommandSwerveController;
import frc.robot.lib6907.CommandSwerveController.DriveMode;
import frc.robot.subsystems.*;
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
  private final TrapFan mTrapFan = new TrapFan();

  private static final boolean kDualController = false;
  private static final boolean isRedAlliance =
      DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Red;

  /* pre-constructed commands */
  private final Command mZeroingCommand = sDrivetrainSubsystem.runZeroingCommand();

  private final DriveWithFollowHeadingCommand mDriveWithRightStick =
      new DriveWithFollowHeadingCommand(
          sDrivetrainSubsystem,
          () ->
              driverController
                  .getDriveTranslation(driverController.isRobotRelative())
                  .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
          () -> readDriveHeading(), // amp heading
          () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED);

  private Optional<Rotation2d> readDriveHeading() {
    Optional<Rotation2d> rightStickHeading = driverController.getDriveRotationAngle();

    // TODO : HEADING KEY BINDINGS HERE

    return rightStickHeading;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    sDrivetrainSubsystem.setDefaultCommand(mDriveWithRightStick);

    sDrivetrainSubsystem.configureAutoBuilder();
    configureBindings();
    pushChooser();
    SmartDashboard.putData(sDrivetrainSubsystem);
    SmartDashboard.putData(mIntake);
    SmartDashboard.putData(mTransfer);
    SmartDashboard.putData(mArm);
    SmartDashboard.putData(mShooter);
    SmartDashboard.putData(mDriveWithRightStick);

    ApriltagCoprocessor.getInstance().setLoggingEnabled(true);
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
     * // Schedule `ExampleCommand` when `exampleCondition` changes to `true` new
     * Trigger(m_exampleSubsystem::exampleCondition) .onTrue(new
     * ExampleCommand(m_exampleSubsystem));
     *
     * // Schedule `exampleMethodCommand` when the Xbox controller's B button is // pressed, //
     * cancelling on release.
     * m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
     */

    /*
     * Swerve
     */

    // Default Command: Drive with right stick

    // reset heading
    Command resetHeadingCommand =
        new InstantCommand(
            () -> {
              sDrivetrainSubsystem.zeroHeading();
              driverController.setTranslationDirection(true);
            });
    resetHeadingCommand.addRequirements(sDrivetrainSubsystem);
    driverController.start().onTrue(resetHeadingCommand);

    // Trigger Rotate
    new Trigger(() -> driverController.getRawRotationRate() != 0.0)
        .onTrue(
            new DriveWithTriggerCommand(
                sDrivetrainSubsystem,
                () ->
                    driverController
                        .getDriveTranslation(driverController.isRobotRelative())
                        .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                () ->
                    driverController.getRawRotationRate()
                        * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, // amp
                // heading
                () -> driverController.isRobotRelative() == DriveMode.ROBOT_ORIENTED));

    // Vision Shoot
    Trigger visionShootTrigger = driverController.y();
    visionShootTrigger
        .whileTrue(
            new VisionShootCommand(
                    mShooter,
                    mArm,
                    mTransfer,
                    sDrivetrainSubsystem,
                    mIntake,
                    () -> driverController.getDriveTranslation(DriveMode.FIELD_ORIENTED))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(
            new InstantCommand(
                () -> {
                  mShooter.stop();
                  mArm.stop();
                  mTransfer.stop();
                  mIntake.stop();
                }));

    // manual trap
    operatorController
        .povUp().and(operatorController.rightBumper())
        .whileTrue(new BlowTrapAndDropCommand(mTrapFan, mShooter, mArm, mTransfer))
        .onFalse(
            new InstantCommand(() -> mShooter.stop())
                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_REST_ANGLE)));
    operatorController
        .povUp().and(operatorController.rightBumper().negate()).whileTrue(new NavTrapCommand(sDrivetrainSubsystem, mArm, mShooter, mIntake, mTransfer, mTrapFan))        .onFalse(
            new InstantCommand(() -> mShooter.stop())
                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_REST_ANGLE)));
    // amp binding
    // navAmp
    buildNavAmpBinding(
        operatorController.povRight().and(operatorController.rightBumper().negate()), isRedAlliance);

    // manual amp
    buildAmpBinding(
        operatorController.povRight().and(operatorController.rightBumper()),
        ShootingParameters.AMP_INTERMEDIATE_POS,
        ShootingParameters.AMP_LOWSPEED);

    // intake system bindings
    if (kDualController) {
      operatorController.a().whileTrue(new IntakeCommand(mIntake, mTransfer));
      operatorController.b().whileTrue(new OuttakeCommand(mIntake, mTransfer));
    } else {
      // chase note inake
      driverController
          .a()
          .and(driverController.rightBumper().negate())
          .whileTrue(new ChaseNoteCommand(sDrivetrainSubsystem, mIntake, mTransfer, mArm));
      // manual intake
      driverController
          .a()
          .and(driverController.rightBumper())
          .whileTrue(new IntakeCommand(mIntake, mTransfer));

      driverController.b().whileTrue(new OuttakeCommand(mIntake, mTransfer));
    }

    // Below Speaker
    if (kDualController) {
      buildShootBinding(operatorController.x(), ShootingParameters.BELOW_SPEAKER);
    } else {
      buildShootBinding(driverController.x(), ShootingParameters.BELOW_SPEAKER);
    }
    // seven zones transfer
    buildPepGBinding(new Trigger[]{driverController.povUp(),driverController.povDown(),driverController.povLeft(),driverController.povRight()});
  }

  private void buildShootBinding(Trigger trigger, ShootingParameters parameters) {
    Command shootCommand =
        new SetShooterTargetCommand(mShooter, parameters.speed_rps)
            .alongWith(new SetArmAngleCommand(mArm, parameters.angle_deg))
            .andThen(new FeedCommand(mTransfer));

    Command stopShootingCommand =
        new InstantCommand(() -> mShooter.stop())
            .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_REST_ANGLE));

    trigger.whileTrue(shootCommand).onFalse(stopShootingCommand);
  }

  private void buildAmpBinding(
      Trigger trigger,
      ShootingParameters IntermediateParameter,
      ShootingParameters targetParameters) {
    Command swingUpCommand =
        new SetShooterTargetCommand(mShooter, targetParameters.speed_rps)
            .alongWith(new SetArmAngleCommand(mArm, IntermediateParameter.angle_deg));

    Command swingBackAndReleaseCommand =
        new SetShooterTargetCommand(mShooter, targetParameters.speed_rps)
            .alongWith(new SetArmAngleCommand(mArm, targetParameters.angle_deg))
            .alongWith(new FeedCommand(mTransfer));

    Command stopShootingCommand =
        new InstantCommand(() -> mShooter.stop())
            .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_REST_ANGLE));

    trigger
        .whileTrue(swingUpCommand.andThen(swingBackAndReleaseCommand))
        .onFalse(stopShootingCommand);
  }

  private void buildPepGBinding(Trigger[] triggers){
    // Command pepGuardiolaCommand = new PepGuardiolaCommand(sDrivetrainSubsystem, mArm, mTransfer, mShooter, null, 
    // ()->triggers[0].getAsBoolean()?goalZones[0]:triggers[1].getAsBoolean()?goalZones[1]:triggers[2].getAsBoolean()?goalZones[2]:goalZones[3]);
    Command pepGuardiolaCommandUP = new PepGuardiolaCommand(sDrivetrainSubsystem, mArm, mTransfer, mShooter,() ->
                    driverController
                        .getDriveTranslation(driverController.isRobotRelative())
                        .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) , GoalZone.UP);
        Command pepGuardiolaCommandDOWN = new PepGuardiolaCommand(sDrivetrainSubsystem, mArm, mTransfer, mShooter,() ->
                    driverController
                        .getDriveTranslation(driverController.isRobotRelative())
                        .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) , GoalZone.DOWN);
        Command pepGuardiolaCommandLEFT = new PepGuardiolaCommand(sDrivetrainSubsystem, mArm, mTransfer, mShooter,() ->
                    driverController
                        .getDriveTranslation(driverController.isRobotRelative())
                        .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) , GoalZone.LEFT);
        Command pepGuardiolaCommandRIGHT = new PepGuardiolaCommand(sDrivetrainSubsystem, mArm, mTransfer, mShooter,() ->
                    driverController
                        .getDriveTranslation(driverController.isRobotRelative())
                        .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) , GoalZone.RIGHT);
    
    triggers[0].onTrue(pepGuardiolaCommandUP);
    triggers[1].onTrue(pepGuardiolaCommandDOWN);
    triggers[2].onTrue(pepGuardiolaCommandLEFT);
    triggers[3].onTrue(pepGuardiolaCommandRIGHT);
  }
  private void buildNavAmpBinding(Trigger trigger, boolean isRedAlliance) {
    Pose2d targetPose =
        isRedAlliance
            ? FieldConstants.IN_FRONT_AMP_POSITION_RED
            : FieldConstants.IN_FRONT_AMP_POSITION_BLUE;
    Command pathfindToAmp =
        AutoBuilder.pathfindToPose(targetPose, PathfindConstants.constraints, 0, 0.5);

    Command swingUpCommand =
        new SetShooterTargetCommand(mShooter, ShootingParameters.AMP_LOWSPEED.speed_rps)
            .alongWith(
                new SetArmAngleCommand(mArm, ShootingParameters.AMP_INTERMEDIATE_POS.angle_deg));

    Command swingBackAndReleaseCommand =
        new SetShooterTargetCommand(mShooter, ShootingParameters.AMP_LOWSPEED.speed_rps)
            .alongWith(new SetArmAngleCommand(mArm, ShootingParameters.AMP_LOWSPEED.angle_deg))
            .alongWith(new FeedCommand(mTransfer));
    Command stopShootingCommand =
        new InstantCommand(() -> mShooter.stop())
            .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_REST_ANGLE));

    trigger
        .whileTrue(pathfindToAmp.alongWith(swingUpCommand).andThen(swingBackAndReleaseCommand))
        .onFalse(stopShootingCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    driverController.setTranslationDirection(true);
    return mChooser.getSelected();
  }

  public void checkDrivetrainZeroing() {
    mZeroingCommand.schedule();
  }

  public void pushChooser() {
    mChooser = new SendableChooser<>();
    mChooser.setDefaultOption("California 51", new CaliforniaAuto(sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake, true));
    mChooser.addOption(
        "California - Start with 51",
        new CaliforniaAuto(sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake, false));
    mChooser.addOption(
        "California - Start with 52",
        new CaliforniaAuto(sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake, true));
    mChooser.addOption("Dallas - Start with 53", 
        new DallasAuto(sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake, mTrapFan, DallasAuto.DallasStrategy.START_53));
    mChooser.addOption("Dallas - Start with 52", 
        new DallasAuto(sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake, mTrapFan, DallasAuto.DallasStrategy.START_52));
    mChooser.addOption("Dallas - Start 53 then Trap", 
        new DallasAuto(sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake, mTrapFan, DallasAuto.DallasStrategy.START_53_THEN_TRAP));
        SmartDashboard.putData("AUTO CHOICES", mChooser);

    SmartDashboard.putData("AUTO CHOICES", mChooser);
  }

  public void moduleTestRoutine() {
    // FL, FR, BR, BL
    var module = sDrivetrainSubsystem.getModuleArray()[2];

    var dv = driverController.getDriveTranslation(DriveMode.ROBOT_ORIENTED);
    module.setDesiredState(new SwerveModuleState(dv.getNorm(), dv.getAngle()));
  }
}
