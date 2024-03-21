package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoyStickCommand extends Command{
    
    private final SwerveSubsystem mSwerveSubsystem;
    private final Supplier<Double> xSpdFunc, ySpdFunc, rotFunc;
    private final Supplier<Boolean> fieldOrientedFunc;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public SwerveJoyStickCommand(SwerveSubsystem SwerveSubsystem, Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> rotFunc, Supplier<Boolean> fieldOrientedFunc) {
        mSwerveSubsystem = SwerveSubsystem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.rotFunc = rotFunc;
        this.fieldOrientedFunc = fieldOrientedFunc;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.rotLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // 1. Get Real-time joystick values
        double xSpeed = xSpdFunc.get();
        double ySpeed = ySpdFunc.get();
        double rotSpeed = rotFunc.get();
        
        //2. Apply deadband
        xSpeed = Math.abs(xSpeed) > DriveConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > DriveConstants.kDeadband ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > DriveConstants.kDeadband ? rotSpeed : 0.0;

        //3. Rate limiter, make the driving experience smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotSpeed = rotLimiter.calculate(rotSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        //4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunc.get()){
            // Relative to the field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, mSwerveSubsystem.getRotation2d());
        }else{
            // Relative to the robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        //5. Convert chassis speeds to invividual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //6. Set the desired states
        mSwerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted){
        mSwerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    

}