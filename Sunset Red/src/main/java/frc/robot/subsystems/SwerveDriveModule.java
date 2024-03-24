package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.SwerveModuleConfig;

public class SwerveDriveModule {
    private static final double AZIMUTH_GEAR_RATIO = 6.0;
    private static final double DRIVE_GEAR_RATIO = 106.0 / 11.0;
    
    public static final double DRIVE_MAXV = 100.0; // rotor rps
    public static final double AZIMUTH_MAXV = 100.0; // rotor rps

    public static final double DRIVE_DEADBAND_MS = 0.1;

    private final TalonFX mDriveMotor;
    private final TalonFX mAzimuthMotor;
    private final DigitalInput mLightGate;
    
    private SwerveModuleConfig mConfig;

    private SwerveModulePosition mModulePosition;

    // Target variables, just for logging purposes
    private double targetDriveVelocityMetersPerSec = 0;
    private double targetSteerPositionRadians = 0;


    public SwerveDriveModule(SwerveModuleConfig config) {
        mDriveMotor = new TalonFX(config.driveID);
        mAzimuthMotor = new TalonFX(config.azimuthID);
        mConfig = config;
        mLightGate = new DigitalInput(config.lightGateID);

        var mAzimuthConfigs = new TalonFXConfiguration();
        mAzimuthConfigs.MotorOutput.Inverted = mConfig.invertAzimuth;
        mAzimuthConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mAzimuthConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        mAzimuthConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        mAzimuthConfigs.Voltage.PeakForwardVoltage = 12.0;
        mAzimuthConfigs.Voltage.PeakReverseVoltage = -12.0;
        mAzimuthConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        mAzimuthConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        mAzimuthConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        mAzimuthConfigs.CurrentLimits.SupplyCurrentLimit = 30.0;
        mAzimuthConfigs.CurrentLimits.SupplyCurrentThreshold = 30.0;
        mAzimuthConfigs.CurrentLimits.SupplyTimeThreshold = 0.0;
        // target error within 0.1
        mAzimuthConfigs.Slot0.kV = mConfig.AZIMUTH_KF;
        mAzimuthConfigs.Slot0.kP = mConfig.AZIMUTH_KP;
        mAzimuthConfigs.Slot0.kI = mConfig.AZIMUTH_KI;
        mAzimuthConfigs.Slot0.kD = mConfig.AZIMUTH_KD;
        mAzimuthConfigs.MotionMagic.MotionMagicJerk = 0.0;
        mAzimuthConfigs.MotionMagic.MotionMagicCruiseVelocity = AZIMUTH_MAXV;
        mAzimuthConfigs.MotionMagic.MotionMagicAcceleration = AZIMUTH_MAXV * 10.0;

        /* !todo Add logging and debugging info here */

        var mDriveConfig = new TalonFXConfiguration();
        mDriveConfig.MotorOutput.Inverted = mConfig.invertDrive;
        mDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mDriveConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        mDriveConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        mDriveConfig.Voltage.PeakForwardVoltage = 12.0;
        mDriveConfig.Voltage.PeakReverseVoltage = -12.0;
        mDriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        mDriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        mDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mDriveConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        mDriveConfig.CurrentLimits.SupplyCurrentThreshold = 30.0;
        mDriveConfig.CurrentLimits.SupplyTimeThreshold = 0.0;
        // target error within 5
        mDriveConfig.Slot0.kV = mConfig.DRIVE_KF;
        mDriveConfig.Slot0.kP = mConfig.DRIVE_KP;
        mDriveConfig.Slot0.kI = mConfig.DRIVE_KI;
        mDriveConfig.Slot0.kD = mConfig.DRIVE_KD;

        mDriveMotor.setPosition(0);

        mModulePosition = new SwerveModulePosition();
    }

    public double getDriveVelocity() {
        double velocityRPS = mDriveMotor.getVelocity().getValueAsDouble();
        double velocityMPS = velocityRPS/DRIVE_GEAR_RATIO*(DriveConstants.kChassisWheelDiameterMeters*Math.PI);
        return velocityMPS; // meters per second
    }

    public double getSteerAngle() {
        double position = mAzimuthMotor.getPosition().getValueAsDouble();
        double angle = position / AZIMUTH_GEAR_RATIO * 2 * Math.PI;
        return angle; // radians
    }

    public double getDrivePosition(){
        return mDriveMotor.getPosition().getValueAsDouble();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),new Rotation2d(getSteerAngle())
        
        );
    }

    public void setTargetState(SwerveModuleState targetState) {
        double currentAngle = getSteerAngle();
        double targetAngle = MathUtil.inputModulus(targetState.angle.getRadians(), 0, 2*Math.PI); // Target angle of the swerve module, limited to a domain between 0 and 2pi

        double absoluteAngle = MathUtil.inputModulus(currentAngle, 0, 2*Math.PI); // Current angle of the swerve module, limited to a domain between 0 and 2pi

        double angleError = MathUtil.inputModulus(targetAngle - absoluteAngle, -Math.PI, Math.PI); // Error between the target angle and the current angle, limited to a domain between -pi and pi

        double resultAngle = currentAngle+angleError; // The result angle of the swerve module

        // setting the target swerve module state values
        // 1. set the velocity of the drive motor
        // 2. set the position of the azimuth motor
        /*
         * setTargetDriveVelocity(targetState.speedMetersPerSecond);
         * setTargetSteerPosition(resultAngle);
         */
        
    }













}
