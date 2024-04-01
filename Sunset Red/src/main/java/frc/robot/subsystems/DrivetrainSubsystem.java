package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.ChassisSpeedKalmanFilterSimplified;

public class DrivetrainSubsystem extends SubsystemBase {

  private final PigeonIMU mPigeon;
  private final SwerveDriveModule[] mSwerveModules;
  private final SwerveDrivePoseEstimator mEstimator;

  private final SwerveDriveKinematics mKinematics;
  private final ChassisSpeedKalmanFilterSimplified mSpeedFilter;

  private ChassisSpeeds mKinematicSpeed;
  private ChassisSpeeds mFilteredSpeed;

  // LogEntries
  private StructLogEntry<Pose2d> mPoseLog;
  private StructLogEntry<ChassisSpeeds> mChassisSpeedLog;
  private StructLogEntry<ChassisSpeeds> mFilteredSpeedLog;

  /*
   * Constructor for DrivetrainSubsystem
   */
  public DrivetrainSubsystem() {

    mPigeon = new PigeonIMU(DriveConstants.kPigeonPort);
    mPigeon.configFactoryDefault();
    // mPigeon.setYaw(0);

    mSwerveModules =
        new SwerveDriveModule[] {
          new SwerveDriveModule(SwerveModuleConstants.FL),
          new SwerveDriveModule(SwerveModuleConstants.FR),
          new SwerveDriveModule(SwerveModuleConstants.BR),
          new SwerveDriveModule(SwerveModuleConstants.BL)
        };

    mKinematics =
        new SwerveDriveKinematics(
            mSwerveModules[0].getTranslationToRobotCenter(),
            mSwerveModules[1].getTranslationToRobotCenter(),
            mSwerveModules[2].getTranslationToRobotCenter(),
            mSwerveModules[3].getTranslationToRobotCenter());

    mSpeedFilter = new ChassisSpeedKalmanFilterSimplified(0.5, 0.5, Constants.kPeriodicDt);

    mEstimator =
        new SwerveDrivePoseEstimator(
            mKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.5, 0.5, 0.5)); // adjust for need (vision-related)

    initLogEntry();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("PoseXMeter", () -> getPose().getX(), null);
    builder.addDoubleProperty("PoseYMeter", () -> getPose().getY(), null);
    builder.addDoubleProperty("PoseAngleDegree", () -> getPose().getRotation().getDegrees(), null);

    mSwerveModules[0].initSendable(builder, getName());
    mSwerveModules[1].initSendable(builder, getName());
    mSwerveModules[2].initSendable(builder, getName());
    mSwerveModules[3].initSendable(builder, getName());
  }

  public void initLogEntry() {
    String name = getName();

    DataLog log = DataLogManager.getLog();
    mPoseLog = StructLogEntry.create(log, name + "/Pose", Pose2d.struct);
    mChassisSpeedLog = StructLogEntry.create(log, name + "/ChassisSpeed", ChassisSpeeds.struct);
    mFilteredSpeedLog = StructLogEntry.create(log, name + "/FilteredSpeed", ChassisSpeeds.struct);

    mSwerveModules[0].initLogEntry(name);
    mSwerveModules[1].initLogEntry(name);
    mSwerveModules[2].initLogEntry(name);
    mSwerveModules[3].initLogEntry(name);
  }

  public void updateLogEntry() {
    mPoseLog.append(getPose());
    mChassisSpeedLog.append(mKinematicSpeed);
    mFilteredSpeedLog.append(mFilteredSpeed);
    // desired speeds ?

    mSwerveModules[0].updateLogEntry();
    mSwerveModules[1].updateLogEntry();
    mSwerveModules[2].updateLogEntry();
    mSwerveModules[3].updateLogEntry();
  }

  /**
   * Drives the robot using the specified translation, rotation, and field-centric mode.
   *
   * @param translation the translation vector representing the robot's desired movement in the
   *     field coordinate system
   * @param rotation the robot's desired rotation rate in radians per second
   * @param fieldCentric a boolean indicating whether the robot should drive in field-centric mode
   *     or not
   */
  public void drive(Translation2d translation, double rotation, boolean fieldCentric) {
    SwerveModuleState[] swerveModuleStates =
        mKinematics.toSwerveModuleStates(
            fieldCentric
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the desired states for each swerve module in the drivetrain. The desired states are
   * specified as an array of SwerveModuleState objects. This method desaturates the wheel speeds
   * based on the maximum physical speed and then sets the desired state for each swerve module.
   *
   * @param desiredStates an array of SwerveModuleState objects representing the desired states for
   *     each swerve module
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    for (int i = 0; i < mSwerveModules.length; i++) {
      mSwerveModules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Returns an array of SwerveModuleState objects representing the current state of each swerve
   * module.
   *
   * @return an array of SwerveModuleState objects
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[mSwerveModules.length];
    for (int i = 0; i < mSwerveModules.length; i++) {
      states[i] = mSwerveModules[i].getState();
    }
    return states;
  }

  /**
   * Returns an array of SwerveModulePosition objects representing the positions of all swerve
   * modules in the drivetrain.
   *
   * @return an array of SwerveModulePosition objects
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveModules.length];
    for (int i = 0; i < mSwerveModules.length; i++) {
      positions[i] = mSwerveModules[i].getPosition();
    }
    return positions;
  }

  /**
   * Returns the current pose of the drivetrain.
   *
   * @return the current pose of the drivetrain
   */
  public Pose2d getPose() {
    return mEstimator.getEstimatedPosition();
  }

  /**
   * Sets the pose of the drivetrain subsystem.
   *
   * @param pose the new pose of the drivetrain subsystem
   */
  public void setPose(Pose2d pose) {
    mEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  /**
   * Returns the heading of the drivetrain subsystem.
   *
   * @return the rotation object representing the heading of the drivetrain subsystem
   */
  private Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Sets the heading of the drivetrain to the specified rotation.
   *
   * @param heading The desired rotation for the drivetrain.
   */
  public void setHeading(Rotation2d heading) {
    mEstimator.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  /**
   * Resets the heading of the drivetrain subsystem to zero. This method updates the position
   * estimator and sets the current pose's rotation to zero.
   */
  public void zeroHeading() {
    mEstimator.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Returns the yaw angle of the gyro in Rotation2d format.
   *
   * @return the yaw angle of the gyro
   */
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(mPigeon.getFusedHeading());
  }

  @Override
  public void periodic() {
    mEstimator.update(getGyroYaw(), getModulePositions());
    mKinematicSpeed =
        mKinematics.toChassisSpeeds(
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState());
    mFilteredSpeed = mSpeedFilter.correctAndPredict(mKinematicSpeed);

    // add logging infomation here
    updateLogEntry();
  }

  private Command runSingleModuleZeroing(SwerveDriveModule module) {
    // [BEAUTIFUL] this does the thing as you read
    return new SequentialCommandGroup(
            Commands.runOnce(module::startZeroing),
            new WaitUntilCommand(module::checkLightGate),
            Commands.runOnce(module::stopAndCalibrate))
        .unless(module::getIsZeroed);
  }

  public Command runZeroingCommand() {
    Command ret =
    // zeroing process: run parallel for 4 modules
        new ParallelCommandGroup(
            runSingleModuleZeroing(mSwerveModules[0]),
            runSingleModuleZeroing(mSwerveModules[1]),
            runSingleModuleZeroing(mSwerveModules[2]),
            runSingleModuleZeroing(mSwerveModules[3]));
    // ADD REQUIREMENT before anything else
    ret.addRequirements(this);
    // run unless already zeroed
    ret = ret.unless(this::allModuleZeroed);
    ret.setName("ZeroingCommand");

    // decorate with no interruption
    return ret.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private boolean allModuleZeroed() {
    return mSwerveModules[0].getIsZeroed()
        && mSwerveModules[1].getIsZeroed()
        && mSwerveModules[2].getIsZeroed()
        && mSwerveModules[3].getIsZeroed();
  }
}
