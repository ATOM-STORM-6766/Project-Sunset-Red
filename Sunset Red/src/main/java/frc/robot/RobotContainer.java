// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.auto.modes.Arizona.ArizonaAutoSweep;
import frc.robot.auto.modes.Arizona.ArizonaAutoSweep.ArizonaSweepMode;
import frc.robot.auto.modes.California.CaliforniaAuto;
import frc.robot.auto.modes.California.CaliforniaAuto.CaliforniaStrategy;
import frc.robot.auto.modes.Dallas.DallasAutoDrop53Routine;
import frc.robot.auto.modes.Dallas.DallasAutoDrop53Routine.Drop53Strategy;
import frc.robot.auto.modes.Dallas.DallasAutoScore53Routine;
import frc.robot.auto.modes.Dallas.DallasAutoScore53Routine.Score53Strategy;
import frc.robot.auto.modes.Dallas.DallasAutoTrap53Routine;
import frc.robot.commands.*;
import frc.robot.commands.PepGuardiolaCommand.GoalZone;
import frc.robot.commands.VisionShootWhileMovingCommand.AimingMode;
import frc.robot.lib6907.CommandSwerveController;
import frc.robot.lib6907.CommandSwerveController.DriveMode;
import frc.robot.subsystems.*;
import frc.robot.utils.ShootingParameters;
import java.util.Optional;
import java.util.function.Function;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private SendableChooser<Command> mChooser = new SendableChooser<>();
        private final SendableChooser<Score53Strategy> mScore53StrategyChooser = new SendableChooser<>();
        private final SendableChooser<Drop53Strategy> mDrop53StrategyChooser = new SendableChooser<>();
        private final SendableChooser<Rotation2d> mFallbackRotation53Chooser = new SendableChooser<>();
        private final SendableChooser<ArizonaSweepMode> mArizonaSweepModeChooser = new SendableChooser<>();
        private final SendableChooser<CaliforniaStrategy> mCaliforniaStrategyChooser = new SendableChooser<>();
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
        private final ApriltagCoprocessor mApriltagCoprocessor = ApriltagCoprocessor.getInstance();
        private final LED mLed = new LED();

        /* pre-constructed commands */
        private final Command mZeroingCommand = sDrivetrainSubsystem.runZeroingCommand();

        private final DriveWithFollowHeadingCommand mDriveWithRightStick = new DriveWithFollowHeadingCommand(
                        sDrivetrainSubsystem,
                        () -> driverController
                                        .getDriveTranslation(driverController.isRobotRelative())
                                        .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                        () -> Optional.empty(), // no more drive with right stick heading
                        () -> false);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                sDrivetrainSubsystem.setDefaultCommand(mDriveWithRightStick);
                mShooter.setDefaultCommand(
                                new SetShooterTargetCommand(mShooter, 0)); // each motor should take only about 3A
                mLed.setDefaultCommand(new LedDefaultCommand(mLed, () -> mTransfer.isOmronDetected()));

                sDrivetrainSubsystem.configureAutoBuilder();
                configureBindings();
                pushChooser();
                SmartDashboard.putData(sDrivetrainSubsystem);
                SmartDashboard.putData(mIntake);
                SmartDashboard.putData(mTransfer);
                SmartDashboard.putData(mArm);
                SmartDashboard.putData(mShooter);
                SmartDashboard.putData(mDriveWithRightStick);
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {

                /*
                 * // Schedule `ExampleCommand` when `exampleCondition` changes to `true` new
                 * Trigger(m_exampleSubsystem::exampleCondition) .onTrue(new
                 * ExampleCommand(m_exampleSubsystem));
                 *
                 * // Schedule `exampleMethodCommand` when the Xbox controller's B button is //
                 * pressed, //
                 * cancelling on release.
                 * m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
                 */

                /*
                 * Swerve
                 */

                // Default Command: Drive with right stick

                // reset heading
                Command resetHeadingCommand = new InstantCommand(
                                () -> {
                                        sDrivetrainSubsystem.zeroHeading();
                                        driverController.setTranslationDirection(true);
                                })
                                .alongWith(new WaitCommand(0.2));
                resetHeadingCommand.addRequirements(sDrivetrainSubsystem);
                driverController.start().onTrue(resetHeadingCommand);

                // Trigger Rotate
                new Trigger(() -> driverController.getRawRotationRate() != 0.0)
                                .onTrue(
                                                new DriveWithTriggerCommand(
                                                                sDrivetrainSubsystem,
                                                                () -> driverController
                                                                                .getDriveTranslation(driverController
                                                                                                .isRobotRelative())
                                                                                .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                                                                () -> driverController.getRawRotationRate()
                                                                                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, // amp
                                                                // heading
                                                                () -> driverController
                                                                                .isRobotRelative() == DriveMode.ROBOT_ORIENTED));

                // Vision Shoot
                Trigger visionShootTrigger = driverController.y();
                visionShootTrigger
                                .whileTrue(
                                                new VisionShootWhileMovingCommand(
                                                                sDrivetrainSubsystem,
                                                                mArm,
                                                                mTransfer,
                                                                mShooter,
                                                                mIntake,
                                                                () -> driverController.getDriveTranslation(
                                                                                DriveMode.FIELD_ORIENTED),
                                                                () -> operatorController.getLeftX(),
                                                                () -> -operatorController.getLeftY(),
                                                                AimingMode.PNP3D)
                                                                .withInterruptBehavior(
                                                                                InterruptionBehavior.kCancelIncoming))
                                .onFalse(
                                                new InstantCommand(
                                                                () -> {
                                                                        mShooter.stop();
                                                                        mTransfer.stop();
                                                                        mIntake.stop();
                                                                })
                                                                .alongWith(new SetArmAngleCommand(mArm,
                                                                                ArmConstants.ARM_OBSERVE_ANGLE)));

                // manual trap
                driverController
                                .povUp()
                                .and(driverController.rightBumper())
                                .whileTrue(new BlowTrapAndDropCommand(mTrapFan, mShooter, mArm, mTransfer))
                                .onFalse(
                                                new InstantCommand(() -> mShooter.stop())
                                                                .andThen(new SetArmAngleCommand(mArm,
                                                                                ArmConstants.ARM_OBSERVE_ANGLE)));
                // nav trap
                driverController
                                .povUp()
                                .and(driverController.rightBumper().negate())
                                .whileTrue(
                                                new NavTrapCommand(sDrivetrainSubsystem, mArm, mShooter, mIntake,
                                                                mTransfer, mTrapFan))
                                .onFalse(
                                                new InstantCommand(() -> mShooter.stop())
                                                                .andThen(new SetArmAngleCommand(mArm,
                                                                                ArmConstants.ARM_OBSERVE_ANGLE)));

                // amp binding
                // navAmp
                driverController
                                .povRight()
                                .and(driverController.rightBumper().negate())
                                .whileTrue(new NavAmpCommand(sDrivetrainSubsystem, mArm, mShooter, mTransfer))
                                .onFalse(new SetArmAngleCommand(mArm, ArmConstants.ARM_OBSERVE_ANGLE));

                // manual amp
                buildAmpBinding(
                                driverController.povRight().and(driverController.rightBumper()),
                                ShootingParameters.AMP_INTERMEDIATE_POS,
                                ShootingParameters.AMP_LOWSPEED);

                buildAmpBinding(operatorController.povRight(),
                                ShootingParameters.AMP_INTERMEDIATE_POS,
                                ShootingParameters.AMP_LOWSPEED);

                // intake system bindings
                // rumble when has note
                Trigger hasNote = new Trigger(() -> mTransfer.isOmronDetected());
                hasNote.onTrue(new InstantCommand(() -> {
                        driverController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
                        driverController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                }).andThen(new WaitCommand(0.2)).andThen(new InstantCommand(() -> {
                        driverController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0);
                        driverController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                })));

                // chase note inake, press left bumper and not pressing right bumper
                driverController
                                .leftBumper()
                                .and(driverController.rightBumper().negate())
                                .whileTrue(new ChaseNoteCommand(sDrivetrainSubsystem, mIntake, mTransfer, mArm)
                                                .andThen( // if end with no note in transfer: we lost target and go do
                                                          // manual intake
                                                                Commands.either(
                                                                                new ParallelCommandGroup(
                                                                                                new IntakeCommand(
                                                                                                                mIntake,
                                                                                                                mTransfer),
                                                                                                new LedBlinkCommand(
                                                                                                                mLed,
                                                                                                                Color.kRed,
                                                                                                                1.0),
                                                                                                // if we dont write
                                                                                                // explicitly drivetrain
                                                                                                // wont run
                                                                                                // because chase note
                                                                                                // requires drivetrain
                                                                                                new DriveWithTriggerCommand(
                                                                                                                sDrivetrainSubsystem,
                                                                                                                () -> driverController
                                                                                                                                .getDriveTranslation(
                                                                                                                                                driverController.isRobotRelative())
                                                                                                                                .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                                                                                                                () -> driverController
                                                                                                                                .getRawRotationRate()
                                                                                                                                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                                                                                                                // heading
                                                                                                                () -> driverController
                                                                                                                                .isRobotRelative() == DriveMode.ROBOT_ORIENTED)),
                                                                                new WaitCommand(0.0),
                                                                                () -> !mTransfer.isOmronDetected())));
                // manual intake, press both left and right bumper
                (driverController.leftBumper().and(driverController.rightBumper()))
                                .or(operatorController.leftBumper())
                                .whileTrue(new ParallelCommandGroup(
                                                new IntakeCommand(mIntake, mTransfer),
                                                Commands.either(
                                                                new SetArmAngleCommand(mArm,
                                                                                ArmConstants.ARM_MAX_INTAKE_ANGLE),
                                                                new WaitCommand(0.0),
                                                                () -> mArm.getAngleDeg() > ArmConstants.ARM_MAX_INTAKE_ANGLE)));
                // outtake
                driverController
                                .b()
                                .or(operatorController.rightBumper())
                                .whileTrue(new OuttakeCommand(mIntake, mTransfer));

                // Shooter Drop
                buildContinuousShootBinding(operatorController.b(), ShootingParameters.DROP);

                // Below Speaker

                buildShootBinding(
                                driverController.x().or(operatorController.x()), ShootingParameters.BELOW_SPEAKER);
                buildShootBinding(
                                operatorController.a(), ShootingParameters.BELOW_SPEAKER_REVERSE);

                // Get note from source
                operatorController
                                .y()
                                .whileTrue(
                                                (new SetShooterTargetCommand(mShooter, -10)
                                                                .alongWith(new SetArmAngleCommand(mArm, 101.0)))
                                                                .until(() -> mTransfer.isOmronDetected())
                                                                .andThen(
                                                                                new InstantCommand(() -> mTransfer
                                                                                                .setVoltage(-1))
                                                                                                .andThen(
                                                                                                                new WaitUntilCommand(
                                                                                                                                () -> !mTransfer.isOmronDetected())
                                                                                                                                .raceWith(new WaitCommand(
                                                                                                                                                0.3)))) // note
                                                                                                                                                        // got
                                                                                                                                                        // in
                                                                .andThen(new InstantCommand(() -> mTransfer.stop())))
                                .onFalse(
                                                new InstantCommand(() -> mTransfer.stop())
                                                                .alongWith(new SetArmAngleCommand(mArm,
                                                                                ArmConstants.ARM_OBSERVE_ANGLE)));

                Trigger rightStickAngle = new Trigger(() -> true);
                rightStickAngle.whileTrue(
                                Commands.run(
                                                () -> SmartDashboard.putNumber(
                                                                "Right Stick Angle",
                                                                driverController
                                                                                .getRightStickToNearestPole()
                                                                                .orElse(new Rotation2d(Math.PI / 4))
                                                                                .getDegrees())));

                // seven zones transfer
                Trigger rightStickUp = new Trigger(
                                () -> driverController
                                                .getRightStickToNearestPole()
                                                .orElse(new Rotation2d(Math.PI / 4))
                                                .getDegrees() == 0.0);

                Trigger rightStickLeft = new Trigger(
                                () -> driverController
                                                .getRightStickToNearestPole()
                                                .orElse(new Rotation2d(Math.PI / 4))
                                                .getDegrees() == 90.0);

                Trigger rightStickRight = new Trigger(
                                () -> driverController
                                                .getRightStickToNearestPole()
                                                .orElse(new Rotation2d(Math.PI / 4))
                                                .getDegrees() == -90.0);

                Trigger rightStickDown = new Trigger(
                                () -> driverController
                                                .getRightStickToNearestPole()
                                                .orElse(new Rotation2d(Math.PI / 4))
                                                .getDegrees() == 180.0);

                buildPepGBinding(new Trigger[] { rightStickUp,
                                rightStickDown.and(driverController.rightBumper().negate()),
                                rightStickLeft, rightStickRight, rightStickDown.and(driverController.rightBumper()) });
        }

        private void buildShootBinding(Trigger trigger, ShootingParameters parameters) {
                Command shootCommand = new SetShooterTargetCommand(mShooter, parameters.speed_rps)
                                .alongWith(new SetArmAngleCommand(mArm, parameters.angle_deg))
                                .andThen(new FeedCommand(mTransfer, mShooter));

                Command stopShootingCommand = new InstantCommand(() -> mShooter.stop())
                                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_OBSERVE_ANGLE));

                trigger.whileTrue(shootCommand).onFalse(stopShootingCommand);
        }

        private void buildContinuousShootBinding(Trigger trigger, ShootingParameters parameters) {
                Command shootCommand = new SetShooterTargetCommand(mShooter, parameters.speed_rps)
                                .alongWith(new SetArmAngleCommand(mArm, parameters.angle_deg))
                                .andThen(new InstantCommand(() -> mTransfer.setVoltage(Transfer.FEED_VOLTS)));

                Command intakeCommand = Commands.run(() -> {
                        if (!mTransfer.isOmronDetected()) {
                                mIntake.setIntake();
                        } else {
                                mIntake.stop();
                        }
                });

                intakeCommand.addRequirements(mIntake, mTransfer);

                Command stopShootingCommand = new InstantCommand(() -> {
                        mShooter.stop();
                        mTransfer.stop();
                        mIntake.stop();
                })
                                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_OBSERVE_ANGLE));

                trigger.whileTrue(shootCommand.alongWith(intakeCommand)).onFalse(stopShootingCommand);
        }

        private void buildAmpBinding(
                        Trigger trigger,
                        ShootingParameters IntermediateParameter,
                        ShootingParameters targetParameters) {
                Command swingUpCommand = new SetShooterTargetCommand(mShooter, targetParameters.speed_rps)
                                .alongWith(new SetArmAngleCommand(mArm, IntermediateParameter.angle_deg));

                Command swingBackAndReleaseCommand = new SetArmAngleCommand(mArm, targetParameters.angle_deg)
                                .alongWith(new FeedCommand(mTransfer, mShooter));

                Command stopShootingCommand = new InstantCommand(() -> mShooter.stop())
                                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_OBSERVE_ANGLE));

                trigger
                                .whileTrue(swingUpCommand.andThen(swingBackAndReleaseCommand))
                                .onFalse(stopShootingCommand);
        }

        private void buildPepGBinding(Trigger[] triggers) {
                Function<GoalZone, PepGuardiolaCommand> buildGuardiola = (goalzone) -> new PepGuardiolaCommand(
                                sDrivetrainSubsystem,
                                mArm,
                                mTransfer,
                                mShooter,
                                mIntake,
                                () -> driverController
                                                .getDriveTranslation(driverController.isRobotRelative())
                                                .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                                goalzone,
                                // TODO : CHECK DIRECTION!
                                () -> operatorController.getHID().getLeftY(),
                                () -> operatorController.getHID().getLeftX());

                PepGuardiolaCommand pepGuardiolaCommandUP = buildGuardiola.apply(GoalZone.UP);
                PepGuardiolaCommand pepGuardiolaCommandDOWN = buildGuardiola.apply(GoalZone.DOWN);
                PepGuardiolaCommand pepGuardiolaCommandHOME = buildGuardiola.apply(GoalZone.HOME);
                PepGuardiolaCommand pepGuardiolaCommandLEFT = buildGuardiola.apply(GoalZone.LEFT);
                PepGuardiolaCommand pepGuardiolaCommandRIGHT = buildGuardiola.apply(GoalZone.RIGHT);

                triggers[0]
                                .whileTrue(
                                                pepGuardiolaCommandUP)
                                .onFalse(new InstantCommand(() -> SmartDashboard.putNumber("Last RightStick", 0)));
                triggers[1]
                                .whileTrue(
                                                pepGuardiolaCommandDOWN)
                                .onFalse(new InstantCommand(() -> SmartDashboard.putNumber("Last RightStick", 180)));
                triggers[2]
                                .whileTrue(
                                                pepGuardiolaCommandLEFT)
                                .onFalse(new InstantCommand(() -> SmartDashboard.putNumber("Last RightStick", 90)));
                triggers[3]
                                .whileTrue(
                                                pepGuardiolaCommandRIGHT)
                                .onFalse(new InstantCommand(() -> SmartDashboard.putNumber("Last RightStick", 270)));
                triggers[4]
                                .whileTrue(
                                                pepGuardiolaCommandHOME)
                                .onFalse(new InstantCommand(() -> SmartDashboard.putNumber("Last RightStick", 180)));

                // led blink when guardiola-DOWN and out of wing
                new Trigger(() -> pepGuardiolaCommandDOWN.isScheduled()
                                && pepGuardiolaCommandDOWN.isOutsideOppositeWing())
                                .onTrue(new LedBlinkCommand(mLed, Color.kGreen, 1.0));
        }

        /** @deprecated use new NavAmpCommand() instead */
        private void buildNavAmpBinding(Trigger trigger, boolean isRedAlliance) {
                Pose2d targetPose = isRedAlliance
                                ? FieldConstants.IN_FRONT_AMP_POSITION_RED
                                : FieldConstants.IN_FRONT_AMP_POSITION_BLUE;
                Command pathfindToAmp = AutoBuilder.pathfindToPose(targetPose, PathfindConstants.constraints, 0, 0.5);

                Command swingUpCommand = new SetShooterTargetCommand(mShooter,
                                ShootingParameters.AMP_LOWSPEED.speed_rps)
                                .alongWith(
                                                new SetArmAngleCommand(mArm,
                                                                ShootingParameters.AMP_INTERMEDIATE_POS.angle_deg));

                Command swingBackAndReleaseCommand = new SetArmAngleCommand(mArm,
                                ShootingParameters.AMP_LOWSPEED.angle_deg)
                                .alongWith(new FeedCommand(mTransfer, mShooter));
                Command stopShootingCommand = new InstantCommand(() -> mShooter.stop())
                                .andThen(new SetArmAngleCommand(mArm, ArmConstants.ARM_OBSERVE_ANGLE));

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
                // Configure Score53Strategy chooser
                mScore53StrategyChooser.setDefaultOption("NEAR", DallasAutoScore53Routine.Score53Strategy.NEAR);
                mScore53StrategyChooser.addOption("FAR", DallasAutoScore53Routine.Score53Strategy.FAR);

                // Configure Drop53Strategy chooser
                mDrop53StrategyChooser.setDefaultOption(
                                "Near Side", DallasAutoDrop53Routine.Drop53Strategy.NEAR_SIDE);
                mDrop53StrategyChooser.addOption("Far Side", DallasAutoDrop53Routine.Drop53Strategy.FAR_SIDE);

                // Configure fallbackRotation53 chooser
                mFallbackRotation53Chooser.setDefaultOption(
                                "90 degrees (intake near)", Rotation2d.fromDegrees(90));
                mFallbackRotation53Chooser.addOption("-90 degrees (intake far)", Rotation2d.fromDegrees(-90));

                // Configure ArizonaSweepMode chooser
                mArizonaSweepModeChooser.setDefaultOption("Near(51to55)", ArizonaSweepMode.NEAR);
                mArizonaSweepModeChooser.addOption("Far(55to51)", ArizonaSweepMode.FAR);

                // Configure CaliforniaStrategy chooser
                mCaliforniaStrategyChooser.setDefaultOption(
                                "Near Side 51, 52, 53", CaliforniaStrategy.DEFAULT);

                // Add Dallas Auto
                mChooser.addOption(
                                "Dallas Score 53 (Configurable)", new ProxyCommand(this::createDallasScore53Command));
                mChooser.addOption(
                                "Dallas Trap 53 (Configurable)", new ProxyCommand(this::createDallasTrap53Command));
                mChooser.addOption(
                                "Dallas Drop 53 (Configurable)", new ProxyCommand(this::createDallasDrop53Command));
                // Add Arizona Auto
                mChooser.addOption(
                                "Arizona Sweep Near", new ProxyCommand(this::createArizonaSweepNearCommand));

                // Add California Auto
                mChooser.addOption(
                                "California Auto Score 51, 52, 53",
                                new ProxyCommand(this::createCaliforniaAutoCommand));

                SmartDashboard.putData("AUTO CHOICES", mChooser);
                SmartDashboard.putData("Score 53 Strategy", mScore53StrategyChooser);
                SmartDashboard.putData("Drop 53 Strategy", mDrop53StrategyChooser);
                SmartDashboard.putData("Fallback Rotation 53", mFallbackRotation53Chooser);
                SmartDashboard.putData("Arizona Sweep Mode", mArizonaSweepModeChooser);
        }

        private Command createArizonaSweepNearCommand() {
                return ArizonaAutoSweep.buildArizonaSweepCommand(
                                sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake,
                                mArizonaSweepModeChooser.getSelected());
        }

        private Command createCaliforniaAutoCommand() {
                return CaliforniaAuto.buildCaliforniaAuto(
                                sDrivetrainSubsystem, mArm, mShooter, mTransfer, mIntake,
                                mCaliforniaStrategyChooser.getSelected());
        }

        private Command createDallasScore53Command() {
                return DallasAutoScore53Routine.buildScore53Command(
                                sDrivetrainSubsystem,
                                mArm,
                                mShooter,
                                mTransfer,
                                mIntake,
                                mScore53StrategyChooser.getSelected(),
                                mFallbackRotation53Chooser.getSelected());
        }

        private Command createDallasTrap53Command() {
                return DallasAutoTrap53Routine.buildTrap53Command(
                                sDrivetrainSubsystem,
                                mArm,
                                mShooter,
                                mTransfer,
                                mIntake,
                                mTrapFan,
                                mFallbackRotation53Chooser.getSelected());
        }

        private Command createDallasDrop53Command() {
                return DallasAutoDrop53Routine.buildDrop53Command(
                                sDrivetrainSubsystem,
                                mArm,
                                mShooter,
                                mTransfer,
                                mIntake,
                                mDrop53StrategyChooser.getSelected(),
                                mFallbackRotation53Chooser.getSelected());
        }

        public void moduleTestRoutine() {
                // FL, FR, BR, BL
                var module = sDrivetrainSubsystem.getModuleArray()[2];

                var dv = driverController.getDriveTranslation(DriveMode.ROBOT_ORIENTED);
                module.setDesiredState(new SwerveModuleState(dv.getNorm(), dv.getAngle()));
        }
}
