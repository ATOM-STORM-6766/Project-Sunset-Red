package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends Command{
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    
    private final double ROTATION_COEFFICIENT = 0.6; // slow down the rotation if not holding the button
    /*
     * Geting the x velocity from the product of the joysticks (Left Y) and the max velocity
     */
    private final DoubleSupplier xVelocitySupplier;

    /* 
     * Getting y velocity from the product of the joysticks (Left X) and the max velocity
     */
    private final DoubleSupplier yVelocitySupplier;

    /*
     * Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity
     */
    private final DoubleSupplier angularVelocitySupplier;

    private final BooleanSupplier robotCentricSupplier;
    

    /**
     * The default drive command constructor
     * 
     * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
     * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the max velocity
     * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the max velocity
     * @param angularVelocitySupplier Gets the joystick value for the angular velocity and multiplies it by the max angular velocity
     */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier angularVelocitySupplier, BooleanSupplier robotCentricSupplier){
        mDrivetrainSubsystem = drivetrainSubsystem;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        addRequirements(drivetrainSubsystem); // required for default command
    }

    @Override
    public void execute(){
        // Running the lambda statements and getting the velocity values
        double xVelocity;
        double yVelocity;
        double angularVelocity;

        xVelocity = xVelocitySupplier.getAsDouble();
        yVelocity = yVelocitySupplier.getAsDouble();
        angularVelocity =
                angularVelocitySupplier.getAsDouble();
        mDrivetrainSubsystem.drive(
            new Translation2d(xVelocity, yVelocity).times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
            angularVelocity*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
            !robotCentricSupplier.getAsBoolean());

    }

    @Override
    public void end(boolean interrupted){
        mDrivetrainSubsystem.drive(new Translation2d(0,0), 0, true);
    }

    @Override
    public boolean isFinished(){
        return false; // Default command never ends
    }
}
