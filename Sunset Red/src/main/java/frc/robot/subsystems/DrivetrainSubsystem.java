package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import javax.sql.rowset.spi.SyncResolver;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public void drive(Translation2d translation, double rotation, boolean fieldCentric){
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            fieldCentric ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        for (int i=0; i< mSwerveModules.length; i++){
            mSwerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /* Used in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i=0; i< mSwerveModules.length; i++){
            mSwerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[mSwerveModules.length];
        for (int i=0; i< mSwerveModules.length; i++){
            states[i] = mSwerveModules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveModules.length];
        for (int i=0; i< mSwerveModules.length; i++){
            positions[i] = mSwerveModules[i].getPosition();
        }
        return positions;
    }

    /**
        * Returns the current pose of the drivetrain.
        *
        * @return the current pose of the drivetrain
        */
        public Pose2d getPose(){
            return mEstimator.getEstimatedPosition();
        }
    
    /**
     * Sets the pose of the drivetrain subsystem.
     * 
     * @param pose the new pose of the drivetrain subsystem
     */
    public void setPose(Pose2d pose){
        mEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    
    
    private Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        mEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        mEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(mPigeon.getYaw().getValue());
    }



    @Override
    public void periodic() {
        
        mEstimator.update(getGyroYaw(), getModulePositions());

        // add logging infomation here
        
    }

}
