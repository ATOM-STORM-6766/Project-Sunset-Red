package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftSteerMotorReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftSteerEncoderOffset);
    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightSteerMotorReversed,
            DriveConstants.kFrontRightAbsoluteEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderReversed,
            DriveConstants.kFrontRightSteerEncoderOffset);
    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftSteerMotorReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderReversed,
            DriveConstants.kBackLeftSteerEncoderOffset);
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightSteerMotorReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderReversed,
            DriveConstants.kBackRightSteerEncoderOffset);

    private final Pigeon2 mGyro = new Pigeon2(0); 

    private final SwerveDriveOdometry mOdometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {

            }
        }).start();
    }

    public void zeroHeading() {
        mGyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(mGyro.getAngle(), 360);
    }


    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return mOdometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        mOdometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        mOdometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //proportionally scales the speeds so that no wheel speed exceeds the max speed
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}
