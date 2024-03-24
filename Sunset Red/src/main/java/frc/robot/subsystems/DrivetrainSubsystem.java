package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import javax.sql.rowset.spi.SyncResolver;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

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
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.config.DrivetrainConfig;


public class DrivetrainSubsystem extends SubsystemBase {

    private static final double WHEELBASE_WIDTH_METERS = DriveConstants.kTrackWidth;
    private static final double WHEELBASE_LENGTH_METERS = DriveConstants.kWheelBase;
    private static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

    private final Pigeon2 mPigeon;

    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

    private final SwerveDriveModule[] mSwerveModules;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;

    private final SwerveModulePosition[] swerveModulePositions;
    private final OdometryUpdateThread OdometryUpdateThread;

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    private final PIDController xController = new PIDController(8, 0, 0);
    private final PIDController yController = new PIDController(8, 0, 0);
    private final PIDController rotationController = new PIDController(1.5, 0, 0);

    /**
     * Thread for updating the odometry,
     * this is done in a separate thread to avoid blocking the main thread
     * and to ensure that the odometry is updated at a consistent rate
     * 
     * 
     */
    private class OdometryUpdateThread extends Thread {
        private final SwerveDriveModule[] mSwerveModules;
        private final Pigeon2 mPigeon;
        private final SwerveDrivePoseEstimator mEstimator;

        public OdometryUpdateThread(SwerveDriveModule[] swerveModules, Pigeon2 pigeon, SwerveDrivePoseEstimator estimator) {
            this.mSwerveModules = swerveModules;
            this.mPigeon = pigeon;
            this.mEstimator = estimator;
        }
        @Override
        public void run(){
            while(!Thread.interrupted()){
                synchronized(mSwerveModules){
                    Rotation2d gyroAngle = Rotation2d.fromDegrees(mPigeon.getYaw().getValue());
                    SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveModules.length];
                    for (int i=0; i< mSwerveModules.length; i++){
                        positions[i] = mSwerveModules[i].getPosition();
                    }
                    mEstimator.update(gyroAngle, positions);
                }
                try{
                    Thread.sleep(20);
                } catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }

    /*
     * Constructor for DrivetrainSubsystem
     */
    public DrivetrainSubsystem() {

        mPigeon = new Pigeon2(DriveConstants.kPigeonPort);

        mSwerveModules = new SwerveDriveModule[] {
                new SwerveDriveModule(SwerveModuleConstants.FL),
                new SwerveDriveModule(SwerveModuleConstants.FR),
                new SwerveDriveModule(SwerveModuleConstants.BR),
                new SwerveDriveModule(SwerveModuleConstants.BL)
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

        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation,
                backRightLocation, backLeftLocation);

        estimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                swerveModulePositions,
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 0.5)); // adjust for need (vision-related)

        OdometryUpdateThread = new OdometryUpdateThread(mSwerveModules, mPigeon, estimator);
        OdometryUpdateThread.start();
    }
    
    /**
        * Returns the rotation in degrees from the gyroscope.
        *
        * @return the rotation in degrees
        */
    private Rotation2d getGyroscopeRotation(){
        return Rotation2d.fromDegrees(mPigeon.getYaw().getValue());
    }

    /**
     * Resets the rotation of the drivetrain subsystem to zero.
     * This method updates the position estimator and sets the rotation of the drivetrain to zero.
     */
    public void zeroRotation() {
        estimator.resetPosition(getGyroscopeRotation(), swerveModulePositions, new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
    }

    /**
        * Returns the current pose of the drivetrain.
        *
        * @return the current pose of the drivetrain
        */
    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }

    /**
     * Sets the pose of the drivetrain subsystem.
     * 
     * @param pose the new pose of the drivetrain subsystem
     */
    public void setPose(Pose2d pose){
        estimator.resetPosition(getGyroscopeRotation(), swerveModulePositions, pose);
    }

    /**
     * Sets the desired drivetrain speed.
     * 
     * @param targetVelocity The target ChassisSpeeds for the drivetrain
     */
    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.targetVelocity = targetVelocity;
    }



    @Override
    public void periodic() {
        
        SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];

        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(targetVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,MAX_SPEED_METERS_PER_SECOND);

        synchronized(mSwerveModules){
            for (int i=0; i< mSwerveModules.length; i++){
                optimizedSwerveModuleStates[i] = SwerveModuleState.optimize(desiredStates[i],
                        mSwerveModules[i].getPosition().angle);
                mSwerveModules[i].setTargetState(optimizedSwerveModuleStates[i]);
            }
        }
        
    }

}
