package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.SwerveModuleConfig;

public class SwerveModule {
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

    public SwerveModule(SwerveModuleConfig config) {
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

    /**
     * Does the math calculation s necessary to get the current swerve module state
     * to the target swerve module state
     * @param targetState : the target swerve module state (contains the target drive velocity and target angle in radians)
     */
    public void setTargetState(SwerveModuleState targetState){
        // todo
    }
    
    public SwerveModulePosition getPosition(){
        //todo
        return mModulePosition; // THIS IS A PLACEHOLDER
    }








}
