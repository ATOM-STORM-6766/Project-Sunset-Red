package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class DrivetrainSubsystem extends SubsystemBase {

    private static final double WHEELBASE_WIDTH_METERS = DriveConstants.kTrackWidth;
    private static final double WHEELBASE_LENGTH_METERS = DriveConstants.kWheelBase;

    private final Pigeon2 mPigeon;
    private final SwerveDriveModule[] mSwerveModules;
    private final SwerveDrivePoseEstimator mEstimator;

    private final SwerveDriveKinematics kinematics;

    /*
     * Constructor for DrivetrainSubsystem
     */
    public DrivetrainSubsystem() {

        mPigeon = new Pigeon2(DriveConstants.kPigeonPort);
        mPigeon.getConfigurator().apply(new Pigeon2Configuration());
        mPigeon.setYaw(0);

        mSwerveModules = new SwerveDriveModule[] {
                new SwerveDriveModule(SwerveModuleConstants.FL),
                new SwerveDriveModule(SwerveModuleConstants.FR),
                new SwerveDriveModule(SwerveModuleConstants.BR),
                new SwerveDriveModule(SwerveModuleConstants.BL)
        };

        var frontLeftLocation = new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEELBASE_WIDTH_METERS / 2);
        var frontRightLocation = new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEELBASE_WIDTH_METERS / 2);
        var backLeftLocation = new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEELBASE_WIDTH_METERS / 2);
        var backRightLocation = new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEELBASE_WIDTH_METERS / 2);

        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation,
                backRightLocation, backLeftLocation);

        mEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 0.5)); // adjust for need (vision-related)
    }

    /**
     * Drives the robot using the specified translation, rotation, and field-centric mode.
     * 
     * @param translation   the translation vector representing the robot's desired movement in the field coordinate system
     * @param rotation      the robot's desired rotation rate in radians per second
     * @param fieldCentric  a boolean indicating whether the robot should drive in field-centric mode or not
     */
    public void drive(Translation2d translation, double rotation, boolean fieldCentric) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldCentric ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getHeading())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        for (int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Sets the desired states for each swerve module in the drivetrain.
     * The desired states are specified as an array of SwerveModuleState objects.
     * This method desaturates the wheel speeds based on the maximum physical speed
     * and then sets the desired state for each swerve module.
     *
     * @param desiredStates an array of SwerveModuleState objects representing the desired states for each swerve module
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    /**
     * Returns an array of SwerveModuleState objects representing the current state of each swerve module.
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
     * Returns an array of SwerveModulePosition objects representing the positions of all swerve modules in the drivetrain.
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
        mEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Resets the heading of the drivetrain subsystem to zero.
     * This method updates the position estimator and sets the current pose's rotation to zero.
     */
    public void zeroHeading() {
        mEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
        * Returns the yaw angle of the gyro in Rotation2d format.
        *
        * @return the yaw angle of the gyro
        */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(mPigeon.getYaw().getValue());
    }

    @Override
    public void periodic() {

        mEstimator.update(getGyroYaw(), getModulePositions());
        // add logging infomation here

    }

}
