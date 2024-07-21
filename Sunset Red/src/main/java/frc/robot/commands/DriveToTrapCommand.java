package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ApriltagCoprocessor;
import frc.robot.subsystems.DrivetrainSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;

public class DriveToTrapCommand extends Command {
    private final DrivetrainSubsystem sDrivetrainSubsystem;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rotationController;

    private Pose2d mTargetPose;
    private boolean mNoTag;

    private static final Transform2d kTagToTargetPose = new Transform2d(0.78, 0, Rotation2d.fromDegrees(0.0));
    private static final int[] RED_TAG_IDS = { 11, 12, 13 }; // Trap tag IDs
    private static final int[] BLUE_TAG_IDS = { 14, 15, 16 }; // Trap tag IDs

    public DriveToTrapCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.sDrivetrainSubsystem = drivetrainSubsystem;

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

        xController = new ProfiledPIDController(5.0, 0, 0, constraints);
        xController.setTolerance(0.05);
        yController = new ProfiledPIDController(5.0, 0, 0, constraints);
        yController.setTolerance(0.05);
        rotationController = new ProfiledPIDController(5.0, 0, 0,
                new TrapezoidProfile.Constraints(Math.toRadians(540.0),
                        Math.toRadians(720.0)));

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(2.0));

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = sDrivetrainSubsystem.getPose();
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        rotationController.reset(currentPose.getRotation().getRadians());

        Optional<Pose2d> tagpose = findVisibleTrapTagPose(ApriltagCoprocessor.getInstance().getShooterSideLatestResult());
        mNoTag = !tagpose.isPresent();
        if (tagpose.isPresent()) {
            mTargetPose = getTargetPose(tagpose.get());
        }
        // todo: log things above
    }

    @Override
    public void execute() {
        Pose2d currentPose = sDrivetrainSubsystem.getPose();
        if (!mNoTag) {
            double xSpeed = xController.calculate(currentPose.getX(), mTargetPose.getX())
                    + xController.getSetpoint().velocity;
            double ySpeed = yController.calculate(currentPose.getY(), mTargetPose.getY())
                    + yController.getSetpoint().velocity;
            double rotationSpeed = rotationController.calculate(
                    currentPose.getRotation().getRadians(),
                    mTargetPose.getRotation().getRadians()) + rotationController.getSetpoint().velocity;

            sDrivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
        } else {
            sDrivetrainSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return (mNoTag) || (xController.atGoal() && yController.atGoal() && rotationController.atGoal());
    }

    private Optional<Pose2d> findVisibleTrapTagPose(PhotonPipelineResult result) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int[] trapIdList = (alliance == Alliance.Blue) ? BLUE_TAG_IDS : RED_TAG_IDS;
        int visible_id = -1;
        for (var usedtag : result.targets) {
            for (int i = 0; i < trapIdList.length; i++)
                if (trapIdList[i] == usedtag.getFiducialId()) {
                    visible_id = usedtag.getFiducialId();
                    break;
                }
            if (visible_id != -1) {
                break;
            }
        }
        if (visible_id == -1) {
            return Optional.empty();
        }

        var tagpose = ApriltagCoprocessor.getInstance().aprilTagFieldLayout.getTagPose(visible_id);
        return Optional.of(tagpose.get().toPose2d());
    }

    private Pose2d getTargetPose(Pose2d trapTagPose) {
        return trapTagPose.plus(kTagToTargetPose);
    }

    public boolean getLastNoTag() {
        return mNoTag;
    }
}