package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.lib6907.CircularInterpolatingTreeMap;
import frc.robot.lib6907.swerve.SwerveKinematicLimits;
import frc.robot.lib6907.swerve.SwerveSetpoint;
import frc.robot.lib6907.swerve.SwerveSetpointGenerator;
import frc.robot.utils.ChassisSpeedKalmanFilterSimplified;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class DrivetrainSubsystem extends SubsystemBase {

  private final Pigeon2 mPigeon;
  private final CircularInterpolatingTreeMap<Double, Rotation2d> mHeading =
      new CircularInterpolatingTreeMap<>(
          100, // Adjust this size as needed
          InverseInterpolator.forDouble(),
          new Interpolator<Rotation2d>() {
            @Override
            public Rotation2d interpolate(Rotation2d startValue, Rotation2d endValue, double t) {
              return startValue.interpolate(endValue, t);
            }
          });

  StructArrayPublisher<SwerveModuleState> swerve_publisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructArrayTopic("MyStates", SwerveModuleState.struct)
          .publish();

  StructArrayPublisher<SwerveModuleState> vision_pose_publisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructArrayTopic("vision pose", SwerveModuleState.struct)
          .publish();
  private final SwerveDriveModule[] mSwerveModules;
  private final SwerveDrivePoseEstimator mEstimator;
  private final Notifier mNotifier;

  StructPublisher<Pose2d> mPosePublisher =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("RobotPose", Pose2d.struct)
          .publish();

  private final SwerveDriveKinematics mKinematics;
  private final ChassisSpeedKalmanFilterSimplified mSpeedFilter;

  private ChassisSpeeds mTargetSpeeds = new ChassisSpeeds();
  private ChassisSpeeds mKinematicSpeed = new ChassisSpeeds();
  private ChassisSpeeds mFilteredSpeed = new ChassisSpeeds();

  // photon vision
  private double photonLatency = 0;

  // LogEntries
  private StructLogEntry<Pose2d> mPoseLog;
  private StructLogEntry<ChassisSpeeds> mChassisSpeedLog;
  private StructLogEntry<ChassisSpeeds> mFilteredSpeedLog;

  private SwerveSetpoint mSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
          });
  private SwerveSetpointGenerator mSetpointGenerator;
  private SwerveKinematicLimits mKinematicLimits =
      new SwerveKinematicLimits(
          DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
          DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 0.05,
          100);

  /*
   * Constructor for DrivetrainSubsystem
   */
  public DrivetrainSubsystem() {

    mPigeon = new Pigeon2(DriveConstants.kPigeonPort);
    Pigeon2Configuration mPigeonConfigurator = new Pigeon2Configuration();
    mPigeon.getConfigurator().apply(mPigeonConfigurator, 10);

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
    mSetpointGenerator = new SwerveSetpointGenerator(mKinematics);
    mSpeedFilter = new ChassisSpeedKalmanFilterSimplified(0.4, 0.4, Constants.kPeriodicDt);

    mEstimator =
        new SwerveDrivePoseEstimator(
            mKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.5, 0.5, 0.5)); // adjust for
    // need
    // (vision-related)

    mNotifier =
        new Notifier(
            () -> {
              synchronized (mEstimator) {
                updateOdom();
              }
            });
    mNotifier.startPeriodic(1.0 / OdometryConstants.kOdomUpdateFreq);

    initLogEntry();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("PoseXMeter", () -> getPose().getX(), null);
    builder.addDoubleProperty("PoseYMeter", () -> getPose().getY(), null);
    builder.addDoubleProperty("PoseAngleDegree", () -> getPose().getRotation().getDegrees(), null);
    builder.addDoubleProperty(
        "ChassisSpeed.omega", () -> mKinematicSpeed.omegaRadiansPerSecond, null);
    builder.addDoubleProperty(
        "FilteredSpeed.omega", () -> mFilteredSpeed.omegaRadiansPerSecond, null);
    builder.addDoubleProperty("ChassisSpeed.vx", () -> mKinematicSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("FilteredSpeed.vx", () -> mFilteredSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("Photon latency", () -> photonLatency, null);
    builder.addDoubleProperty("Last Vision Update TIme", () -> lastVisionOdomUpdateTime, null);
    builder.addDoubleArrayProperty(
        "target chassis speeds x y omega",
        () ->
            new double[] {
              mTargetSpeeds.vxMetersPerSecond,
              mTargetSpeeds.vyMetersPerSecond,
              mTargetSpeeds.omegaRadiansPerSecond
            },
        null);
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

  private void genSetpointAndApply(ChassisSpeeds des_cs) {
    final Pose2d kPose2dIdentity = new Pose2d();
    final double kLooperFreq = 1.0 / Constants.kPeriodicDt;

    Pose2d robot_pose_vel =
        new Pose2d(
            des_cs.vxMetersPerSecond * Constants.kPeriodicDt,
            des_cs.vyMetersPerSecond * Constants.kPeriodicDt,
            Rotation2d.fromRadians(des_cs.omegaRadiansPerSecond * Constants.kPeriodicDt));
    Twist2d twist_vel = kPose2dIdentity.log(robot_pose_vel);
    ChassisSpeeds updated_chassis_speeds =
        new ChassisSpeeds(
            twist_vel.dx * kLooperFreq, twist_vel.dy * kLooperFreq, twist_vel.dtheta * kLooperFreq);
    mTargetSpeeds = updated_chassis_speeds;
    mSetpoint =
        mSetpointGenerator.generateSetpoint(
            mKinematicLimits, mSetpoint, updated_chassis_speeds, Constants.kPeriodicDt, this);
    setModuleStates(mSetpoint.mModuleStates);
  }

  /**
   * Drives the robot using the specified translation, rotation, and field-centric mode.
   *
   * @param translation the translation vector representing the robot's desired movement in the
   *     field coordinate system in meters per second
   * @param rotation the robot's desired rotation rate in radians per second
   * @param fieldCentric a boolean indicating whether the robot should drive in field-centric mode
   *     or not
   */
  public void drive(Translation2d translation, double rotation, boolean fieldCentric) {
    genSetpointAndApply(
        fieldCentric
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  }

  public void driveWithChassisSpeed(ChassisSpeeds mChassisSpeeds) {
    genSetpointAndApply(mChassisSpeeds);
  }

  /**
   * Sets the desired states for each swerve module in the drivetrain. The desired states are
   * specified as an array of SwerveModuleState objects. This method desaturates the wheel speeds
   * based on the maximum physical speed and then sets the desired state for each swerve module.
   *
   * @param desiredStates an array of SwerveModuleState objects representing the desired states for
   *     each swerve module
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    swerve_publisher.set(desiredStates);

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

  private double compensate(
      StatusSignal<Double> pos_sig, StatusSignal<Double> pos_vel, double ctretime) {
    final double nonCompensatedSignal = pos_sig.getValueAsDouble();
    final double changeInSignal = pos_vel.getValueAsDouble();
    double latency = ctretime - pos_sig.getTimestamp().getTime();
    return nonCompensatedSignal + (changeInSignal * latency);
  }

  /**
   * Returns the current pose of the drivetrain.
   *
   * @return the current pose of the drivetrain
   */
  public Pose2d getPose() {
    synchronized (mEstimator) {
      return mEstimator.getEstimatedPosition();
    }
  }

  /***
   * Returns the filtered velocity of the drivetrain in m/s
   *
   * @param pose the filtered velocity of the drivetrain
   */
  public Translation2d getVelocity() {
    return new Translation2d(mFilteredSpeed.vxMetersPerSecond, mFilteredSpeed.vyMetersPerSecond);
  }

  /**
   * Sets the pose of the drivetrain subsystem.
   *
   * @param pose the new pose of the drivetrain subsystem
   */
  public void setPose(Pose2d pose) {
    synchronized (mEstimator) {
      mEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
  }

  /**
   * Returns the heading of the drivetrain subsystem.
   *
   * @return the rotation object representing the heading of the drivetrain subsystem
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Sets the heading of the drivetrain to the specified rotation.
   *
   * @param heading The desired rotation for the drivetrain.
   */
  public void setHeading(Rotation2d heading) {
    synchronized (mEstimator) {
      mEstimator.resetPosition(
          getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }
  }

  /**
   * Resets the heading of the drivetrain subsystem to zero. This method updates the position
   * estimator and sets the current pose's rotation to zero.
   */
  public void zeroHeading() {
    synchronized (mEstimator) {
      mEstimator.resetPosition(
          getGyroYaw(),
          getModulePositions(),
          new Pose2d(
              getPose().getTranslation(),
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                  ? new Rotation2d()
                  : Rotation2d.fromDegrees(180)));
    }
  }

  /**
   * Returns the yaw angle of the gyro in Rotation2d format.
   *
   * @return the yaw angle of the gyro
   */
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(
        -mPigeon
            .getAngle()); // pigeon 2 uses NED tradition, clockwise is positive rotation, but we use
    // yaw tradition
  }

  double lastVisionOdomUpdateTime = Timer.getFPGATimestamp();

  @Override
  public void periodic() {

    mKinematicSpeed =
        mKinematics.toChassisSpeeds(
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState());
    mFilteredSpeed = mSpeedFilter.correctAndPredict(mKinematicSpeed);
    // add logging infomation here
    updateLogEntry();

    // if(robotStationary()){
    lastVisionOdomUpdateTime = updateOdomFromVision(); // does not change value if not update from
    // vision.
    // }
    mPosePublisher.set(getPose());
  }

  public boolean robotStationary() {
    return Math.abs(mFilteredSpeed.vxMetersPerSecond) < 0.2
        && Math.abs(mFilteredSpeed.vyMetersPerSecond) < 0.2
        && Math.abs(mFilteredSpeed.omegaRadiansPerSecond) < 0.2;
  }

  private double updateOdomFromVision() {
    synchronized (mEstimator) {
      Optional<EstimatedRobotPose> visionEstimatedPose =
          ApriltagCoprocessor.getInstance()
              .updateEstimatedGlobalPose(
                  mEstimator.getEstimatedPosition(),
                  new Translation2d(
                      mFilteredSpeed.vxMetersPerSecond, mFilteredSpeed.vyMetersPerSecond));

      if (visionEstimatedPose.isPresent()) {
        Pose2d estimatedPose2d = visionEstimatedPose.get().estimatedPose.toPose2d();
        double photonTimestamp = visionEstimatedPose.get().timestampSeconds;
        double currentTimestamp = Timer.getFPGATimestamp();
        photonLatency = currentTimestamp - photonTimestamp;
        Pose2d useIMUPose2d =
            new Pose2d(estimatedPose2d.getTranslation(), mHeading.get(photonTimestamp));
        if (useIMUPose2d.getRotation() != null) {
          if (visionEstimatedPose.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            mEstimator.addVisionMeasurement(
                estimatedPose2d, photonTimestamp, VecBuilder.fill(0.1, 0.1, 0.1));
          } else {
            mEstimator.addVisionMeasurement(
                useIMUPose2d, photonTimestamp, VecBuilder.fill(0.25, 0.25, 0.25));
          }
          return Timer.getFPGATimestamp();
        }
      }

      // mHeading.clear(); no longer needs to clear as it is circular buffer
      return lastVisionOdomUpdateTime;
    }
  }

  private void updateOdom() {
    synchronized (mEstimator) {
      SwerveModulePosition[] mModulePositions = new SwerveModulePosition[mSwerveModules.length];
      double timestamp = Timer.getFPGATimestamp();
      double ctretime = Utils.getCurrentTimeSeconds();
      for (int i = 0; i < mSwerveModules.length; i++) {
        mModulePositions[i] = new SwerveModulePosition();
        SwerveDriveModule module = mSwerveModules[i];
        double driverot =
            compensate(module.getDrivePositionSignal(), module.getDriveVelocitySignal(), ctretime);
        mModulePositions[i].distanceMeters =
            driverot * DriveConstants.kChassisWheelCircumferenceMeters;
        double anglerot =
            compensate(
                module.getAzimuthPositionSignal(), module.getAzimuthVelocitySignal(), ctretime);
        anglerot = anglerot - module.getAzimuthOffsetRotations();
        mModulePositions[i].angle = Rotation2d.fromDegrees(anglerot * 360);
      }
      mEstimator.updateWithTime(timestamp, getGyroYaw(), mModulePositions);
      mHeading.put(timestamp, getHeading());
    }
  }

  private Command runSingleModuleZeroing(SwerveDriveModule module) {
    // [BEAUTIFUL] this does the thing as you read
    return new SequentialCommandGroup(
            Commands.runOnce(module::startZeroing),
            new WaitUntilCommand(module::checkLightGate),
            Commands.runOnce(module::stopAndCalibrate))
        .unless(module::getIsZeroed);
  }

  private static final SwerveModuleState[] kNeutralStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
      };

  public void stop() {
    setModuleStates(kNeutralStates);
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

  public boolean allModuleZeroed() {
    return mSwerveModules[0].getIsZeroed()
        && mSwerveModules[1].getIsZeroed()
        && mSwerveModules[2].getIsZeroed()
        && mSwerveModules[3].getIsZeroed();
  }

  public SwerveDriveModule[] getModuleArray() {
    return mSwerveModules;
  }

  public void configureAutoBuilder() {
    if (AutoBuilder.isConfigured()) return;

    AutoBuilder.configureHolonomic(
        this::getPose, // poseSupplier
        this::setPose, // resetPose
        () -> mKinematicSpeed, // robotRelativeSpeedsSupplier
        this::driveWithChassisSpeed, // robotRelativeOutput
        new HolonomicPathFollowerConfig(
            // translationConstants: PPLib uses wpilib PIDController
            // input is pose error in meters, output is compensated velocity in meters per second
            new PIDConstants(5.0, 0.0, 0.0),
            // rotationConstants: PPLib uses wpilib PIDController
            // input is angle error in radius, output is compensated rotation in radians per second
            new PIDConstants(5.0, 0.0, 0.0),
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond, // maxModuleSpeed
            DriveConstants.kWheelBase / Math.sqrt(2), // driveBaseRadius
            new ReplanningConfig(), // replanningConfig (how and when should a path be replanned)
            Constants.kPeriodicDt // period for pid update, KEEP SYNC WITH ROBOT PERIOD
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }
}
