package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import javax.sql.rowset.spi.SyncResolver;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.DrivetrainConfig;
import frc.robot.subsystems.GyroIO.GyroIOInputs;

public class DrivetrainSubsystem extends SubsystemBase {

    private static final double WHEELBASE_WIDTH_METERS = DriveConstants.kTrackWidth;
    private static final double WHEELBASE_LENGTH_METERS = DriveConstants.kWheelBase;
    private static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

    private final GyroIO mGyroIO;
    private final GyroIO.GyroIOInputs gyroInputs = new GyroIO.GyroIOInputs();
    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

    private final SwerveModule[] mSwerveModules;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;

    private final SwerveModulePosition[] swerveModulePositions;

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    private final PIDController xController = new PIDController(8, 0, 0);
    private final PIDController yController = new PIDController(8, 0, 0);
    private final PIDController rotationController = new PIDController(1.5, 0, 0);

    /*
     * Constructor for DrivetrainSubsystem
     */
    public DrivetrainSubsystem(
            GyroIO gyroIO,
            SwerveModuleIO frontLeftModuleIO,
            SwerveModuleIO frontRightModuleIO,
            SwerveModuleIO backLeftModuleIO,
            SwerveModuleIO backRightModuleIO,
            DrivetrainConfig drivetrainConfig) {

        this.mGyroIO = gyroIO;

        mSwerveModules = new SwerveModule[] {
                new SwerveModule(frontLeftModuleIO, "FrontLeft"),
                new SwerveModule(frontRightModuleIO, "FrontRight"),
                new SwerveModule(backLeftModuleIO, "BackLeft"),
                new SwerveModule(backRightModuleIO, "BackRight")
        };

        swerveModulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        var frontLeftLocation = new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEELBASE_WIDTH_METERS / 2);
        var frontRightLocation = new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEELBASE_WIDTH_METERS / 2);
        var backLeftLocation = new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEELBASE_WIDTH_METERS / 2);
        var backRightLocation = new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEELBASE_WIDTH_METERS / 2);

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
                backRightLocation);

        odometry = new SwerveDriveOdometry(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                },
                new Pose2d());
        estimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 0.5));
    }

    @Override
    public void periodic() {
        mGyroIO.updateInputs(gyroInputs);
        // !todo Log gyro readings
        // placeholder here

        SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];

        /*
         * Use the target chassis speed to calculate angle and speed for each
         * swerve drive module
         */
        swerveModuleStates = kinematics.toSwerveModuleStates(targetVelocity);

        /*
         * Make sure the wheel speeds are within the limits of the physical robot
         */
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_METERS_PER_SECOND);

        /*
         * Make sure that the swerve module wheel can get to the desired state in the
         * fastest way
         * Linking this with the PIDController's continous inout means that the module's
         * steer motor
         * will never turn more than 90 degrees in either direction
         */
        synchronized (mSwerveModules) {
            for (int i = 0; i < optimizedSwerveModuleStates.length; i++) {
                optimizedSwerveModuleStates[i] = SwerveModuleState.optimize(swerveModuleStates[i],
                        mSwerveModules[i].getPosition().angle);
                mSwerveModules[i].setTargetState(optimizedSwerveModuleStates[i]);

            }
        }

        // !todo: add logging for everything

        synchronized (estimator) {
            // !todo add log here
            // place holder
            System.out.println("placeholder for estimator logging");
        }

        synchronized (odometry) {
            // !todo add log here
            // place holder
            System.out.println("placeholder for odometry logging");
        }


    }

    /**
     * Sets the desired drivetrain speed. 
     * 
     * @param targetVelocity The target ChassisSpeeds for the drivetrain
     */
    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public Pose2d getPose(){
        synchronized (odometry){
            return odometry.getPoseMeters();
        }
    }

    public double getAngularVelocity(){
        synchronized (gyroInputs){
            return gyroInputs.angularVelocity;
        }
    }

}
