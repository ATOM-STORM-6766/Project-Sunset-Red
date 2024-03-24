package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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

    private final BooleanSupplier joystickButton;
    

    /**
     * The default drive command constructor
     * 
     * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules
     * @param xVelocitySupplier Gets the joystick value for the x velocity and multiplies it by the max velocity
     * @param yVelocitySupplier Gets the joystick value for the y velocity and multiplies it by the max velocity
     * @param angularVelocitySupplier Gets the joystick value for the angular velocity and multiplies it by the max angular velocity
     */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier angularVelocitySupplier, BooleanSupplier joystickButton){
        mDrivetrainSubsystem = drivetrainSubsystem;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        this.joystickButton = joystickButton;
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
                angularVelocitySupplier.getAsDouble() * (joystickButton.getAsBoolean() ? 1.0 : ROTATION_COEFFICIENT);
        
        // Log here 
        // place holder for logging
        
        // Create a new Chassis speed object with the velocity values
        ChassisSpeeds targetVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity,
                mDrivetrainSubsystem
                        .getPose()
                        .getRotation());
        // blue oriented problem, ... 
        // SwerveController from 6907 2024 code ... !todo, 扔到robotcontainer里

        // Setting the robot velocity to chassis velocity
        mDrivetrainSubsystem.setTargetVelocity(targetVelocity);

    }

    @Override
    public void end(boolean interrupted){
        mDrivetrainSubsystem.setTargetVelocity(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished(){
        return false; // Default command never ends
    }
}
