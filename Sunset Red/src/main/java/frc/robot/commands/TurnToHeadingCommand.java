package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToHeadingCommand extends Command {

    private final DrivetrainSubsystem sDrivetrainSubsystem;

    private Rotation2d mTargetHeading;
    // input in radians and output in rad/s
    private final ProfiledPIDController mProfiledPID = new ProfiledPIDController(0.5, 0, 0.0,
            new TrapezoidProfile.Constraints(
                    Math.toRadians(540), Math.toRadians(720)));

    private static final Translation2d kZeroTranslation = new Translation2d();

    // turn to given heading once and finished when target reached
    public TurnToHeadingCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Rotation2d targetHeading) {
        sDrivetrainSubsystem = drivetrainSubsystem;
        mTargetHeading = targetHeading;
        addRequirements(sDrivetrainSubsystem);
    }

    public TurnToHeadingCommand withPID(double kP, double kI, double kD) {
        mProfiledPID.setPID(kP, kI, kD);
        return this;
    }

    @Override
    public void initialize() {
        mProfiledPID.reset(sDrivetrainSubsystem.getHeading().getRadians());
        mProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
        mProfiledPID.setGoal(mTargetHeading.getRadians());
        mProfiledPID.setTolerance(Math.toRadians(0.5));
    }

    @Override
    public void execute() {
        // in rad/s
        double turnspeed = mProfiledPID.calculate(sDrivetrainSubsystem.getHeading().getRadians())
                + mProfiledPID.getSetpoint().velocity;
        sDrivetrainSubsystem.drive(kZeroTranslation, turnspeed, true);
    }

    @Override
    public boolean isFinished() {
        return mProfiledPID.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
