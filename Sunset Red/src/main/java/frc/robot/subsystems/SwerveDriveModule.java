package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.SwerveModuleConfig;

public class SwerveDriveModule extends SubsystemBase {
    private static final double AZIMUTH_GEAR_RATIO = 6.0;
    private static final double DRIVE_GEAR_RATIO = 106.0 / 11.0;

    public static final double DRIVE_MAXV = 100.0; // rotor rps
    public static final double AZIMUTH_MAXV = 100.0; // rotor rps

    public static final double DRIVE_DEADBAND_MS = 0.1;

    private final TalonFX mDriveMotor;
    private final TalonFX mAzimuthMotor;
    private final DigitalInput mLightGate;

    private SwerveModuleConfig mConfig;

    private final SimpleMotorFeedforward driveFeadforward = new SimpleMotorFeedforward(DriveConstants.kDriveKS,
            DriveConstants.kDriveKV, DriveConstants.kDriveKA);

    /* Drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* Azimuth motor control requests */
    private final PositionVoltage azimuthPosition = new PositionVoltage(0);

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

    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAzimuthMotor.setControl(azimuthPosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {

        /*
         * COnvert MPS to RPS, adjusting for the drive gear ratio
         */
        double velocityRPS = desiredState.speedMetersPerSecond / DriveConstants.kChassisWheelDiameterMeters * Math.PI
                * DRIVE_GEAR_RATIO;

        driveVelocity.Velocity = velocityRPS;
        driveVelocity.FeedForward = driveFeadforward.calculate(desiredState.speedMetersPerSecond);
        mDriveMotor.setControl(driveVelocity);
    }

    public SwerveModuleState getState() {

        // Convert RPS to MPS, adjusting for the drive gear ratio.
        double velocityMPS = mDriveMotor.getVelocity().getValue() * DriveConstants.kChassisWheelDiameterMeters * Math.PI
                / DRIVE_GEAR_RATIO;

        return new SwerveModuleState(
                velocityMPS,
                Rotation2d.fromRotations(mAzimuthMotor.getPosition().getValue()));
    }

    public SwerveModulePosition getPosition(){
    
        // Convert motor rotations to meters, adjusting for the drive gear ratio.
        // This assumes DRIVE_GEAR_RATIO defines motor rotations per wheel rotation.
        double distanceMeters = mDriveMotor.getPosition().getValue() * DriveConstants.kChassisWheelDiameterMeters * Math.PI / DRIVE_GEAR_RATIO;
    
        return new SwerveModulePosition(
            distanceMeters, 
            Rotation2d.fromRotations(mAzimuthMotor.getPosition().getValue())
        );
    }

}
