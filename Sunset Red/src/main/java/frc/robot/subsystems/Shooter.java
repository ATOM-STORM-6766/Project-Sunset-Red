package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib6907.DelayedBoolean;
import frc.robot.utils.Util;

public class Shooter extends SubsystemBase{
    private static final double SHOOT_THRESHOLD_RPS = 20.0; // lower than this velocity, game piece will get stuck
    private static final double ERR_TOL = 2.0; // spin velocity error tolerance (rps)
    private static final double STABLIZE_TIME = 0.1;

    private final TalonFX mShooterTalon;
    private final TalonFX mShooterFollower;

    private DelayedBoolean mSpinStablized;

    


    private PeriodicIO mPeriodicIO;
    private static class PeriodicIO {
        // INPUTS
        public double shooterVelocity = 0.0, followerVelocity = 0.0; // rps
        public double shooterVoltage = 0.0; // volts

        // OUTPUTS
        public double shooterTargetRPS = 0.0;
        public final VelocityVoltage shooterTargetVelocity = new VelocityVoltage(0);
    }

    private DoubleLogEntry mVelocityLog;
    private DoubleLogEntry mTargetVelocityLog;

    public Shooter(){
        mShooterTalon = new TalonFX(ShooterConstants.SHOOTER_ID);
        mShooterFollower = new TalonFX(ShooterConstants.SHOOTER_FOLLOWER);
        mSpinStablized = new DelayedBoolean(0.0, STABLIZE_TIME);
        mPeriodicIO = new PeriodicIO();

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        shooterConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterConfig.Voltage.PeakReverseVoltage = -12.0;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooterConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
        shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        // TODO : TUNE SHOOTER VEL PID
        shooterConfig.Slot0.kV = 0.113;
        shooterConfig.Slot0.kP = 0.1;
        shooterConfig.Slot0.kI = 1.0;
        shooterConfig.Slot0.kD = 0.0;
        Util.checkReturn("shooter", mShooterTalon.getConfigurator().apply(shooterConfig, Constants.kLongCANTimeoutSec));
        // TODO : TUNE SHOOTER FOLLOWER VEL PID
        shooterConfig.Slot0.kV = 0.118;
        shooterConfig.Slot0.kP = 0.1;
        shooterConfig.Slot0.kI = 1.0;
        shooterConfig.Slot0.kD = 0.0;
        Util.checkReturn("shooter follower", mShooterFollower.getConfigurator().apply(shooterConfig, Constants.kLongCANTimeoutSec));
        mShooterTalon.setControl(Constants.NEUTRAL);
        mShooterFollower.setControl(Constants.NEUTRAL);
        Util.checkReturn("shooter velocity", mShooterTalon.getVelocity().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
        Util.checkReturn("shooter follower velocity", mShooterFollower.getVelocity().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
        Util.checkReturn("shooter voltage", mShooterTalon.getMotorVoltage().setUpdateFrequency(100, Constants.kLongCANTimeoutSec));
        Util.checkReturn("shooter canbus", mShooterTalon.optimizeBusUtilization(Constants.kLongCANTimeoutSec));
        Util.checkReturn("shooter follower canbus", mShooterFollower.optimizeBusUtilization(Constants.kLongCANTimeoutSec));

        initLogEntry();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Shooter Velocity", ()->mPeriodicIO.shooterVelocity, null);
        builder.addDoubleProperty("Shooter Target RPS", ()->mPeriodicIO.shooterTargetRPS, null);
    }

    /** */
    private void initLogEntry() {
        String name = getName();

        DataLog log = DataLogManager.getLog();
        
        mVelocityLog = new DoubleLogEntry(log, name+"/Velocity");
        mTargetVelocityLog = new DoubleLogEntry(log, name+"/TargetVelocity");
    }

    /**
     * @brief add new information to logger
     */
    private void updateLogEntry() {
        mVelocityLog.append(getAverageVelocity());
        mTargetVelocityLog.append(mPeriodicIO.shooterTargetRPS);
    }

    /**
     * @brief Set the target velocity in rps of the shooter. The shooter will kepp this target until next call. 
     * @param target_rps target velocity in rotations per second
     */
    public void setTargetVelocity(double target_rps){
        mPeriodicIO.shooterTargetRPS = target_rps;
    }

    /**
     * @brief a way to tell the shooter to stop (apply neutral out) a wrapper for `setTargetVelocity(0)`
     */
    public void stop(){
        setTargetVelocity(0);
    }

    /**
     * @brief returns whether the shooter is ready to shoot
     * @return
     */
    public boolean isReadyToShoot(){
        return mSpinStablized.update(Timer.getFPGATimestamp(), 
           mPeriodicIO.shooterVelocity > SHOOT_THRESHOLD_RPS && mPeriodicIO.followerVelocity > SHOOT_THRESHOLD_RPS
        && Math.abs(mPeriodicIO.shooterVelocity  - mPeriodicIO.shooterTargetRPS) < ERR_TOL
        && Math.abs(mPeriodicIO.followerVelocity - mPeriodicIO.shooterTargetRPS) < ERR_TOL);
    }

    public double getAverageVelocity(){
        return mPeriodicIO.shooterVelocity * 0.5 + mPeriodicIO.followerVelocity * 0.5;
    }

    public void readPeriodicInputs() {
        mPeriodicIO.shooterVelocity = mShooterTalon.getVelocity().getValueAsDouble();
        mPeriodicIO.followerVelocity = mShooterFollower.getVelocity().getValueAsDouble();
        mPeriodicIO.shooterVoltage = mShooterTalon.getMotorVoltage().getValueAsDouble();
    }

    public void writePeriodicOutputs(){
        if(mPeriodicIO.shooterTargetRPS == 0.0){
            // prevent repeatedly creating NeutralOut instance
            mShooterTalon.setControl(Constants.NEUTRAL);
            mShooterFollower.setControl(Constants.NEUTRAL);
        }else{
            mPeriodicIO.shooterTargetVelocity.Velocity = mPeriodicIO.shooterTargetRPS;
            mShooterTalon.setControl(mPeriodicIO.shooterTargetVelocity);
            mShooterFollower.setControl(mPeriodicIO.shooterTargetVelocity);
        }
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
        updateLogEntry();
    }
}
