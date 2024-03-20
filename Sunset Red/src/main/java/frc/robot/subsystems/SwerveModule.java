package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final CANCoder driveEncoder;
    private final CANCoder steerEncoder;

    private final PIDController steerPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorInversed,
            int driveEncoderId, int steerEncoderId, int absoluteEncoderId, boolean absoluteEncoderReversed,
            double absoluteEncoderOffsetRad) {
        driveMotor = new TalonFX(driveMotorId);
        steerMotor = new TalonFX(steerMotorId);

        driveEncoder = new CANCoder(driveEncoderId);
        steerEncoder = new CANCoder(steerEncoderId);

        steerPidController = new PIDController(SwerveModuleConstants.kPsteer, 0, 0);

        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorInversed);

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSecond);
        steerEncoder.setPositionConversionFactor(SwerveModuleConstants.kSteerEncoderRot2Rad);
        steerEncoder.setVelocityConversionFactor(SwerveModuleConstants.kSteerEncoderRPM2RadPerSecond);

        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= -2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        /*
         * This function is intended to solve the problem
         * of whenever you let go of the joystick, the WPILib
         * will reset the module states to zero
         */
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.set(steerPidController.calculate(getSteerPosition(), state.angle.getRadians()));
        // add some logging here

    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

}
