package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommandGroups.ZeroAndShootPreloadCommand;
import frc.robot.subsystems.ApriltagCoprocessor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class CenterLineAuto extends Command{

    

    private DrivetrainSubsystem sDrivetrainSubsystem;
    private Intake sIntake;
    private Transfer sTransfer;
    private Shooter sShooter;
    private Arm sArm;
    private GamePieceProcessor sGamePieceProcessor;
    private ApriltagCoprocessor sApriltagCoprocessor;

    public enum AutoState {
        ZEROING_AND_SHOOT_PRELOAD, // drive train zeroing
        SEARCHING_NOTE, // no note in robot, no note seen
        CHASING_NOTE, // no note in robot, but note seen
        RETURN_AND_SHOOT, // has note in robot, going back to shoot
    }

    public AutoState mNextState = AutoState.ZEROING_AND_SHOOT_PRELOAD;
    public AutoState mCurrentState = AutoState.ZEROING_AND_SHOOT_PRELOAD;

    public ZeroAndShootPreloadCommand mZeroAndShootPreloadCommand;
    public ChaseNoteCommand mChaseNoteCommand;

    private static final Pose2d kCenterHome = new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(0.0));

    
    public CenterLineAuto(DrivetrainSubsystem drivetrainSubsystem, Intake intake, Transfer transfer, Shooter shooter, Arm arm, GamePieceProcessor gamePieceProcessor, ApriltagCoprocessor apriltagCoprocessor){
        sDrivetrainSubsystem = drivetrainSubsystem;
        sIntake = intake;
        sTransfer = transfer;
        sShooter = shooter;
        sArm = arm;
        sApriltagCoprocessor = apriltagCoprocessor;
        sGamePieceProcessor = gamePieceProcessor;

        mZeroAndShootPreloadCommand = new ZeroAndShootPreloadCommand(sDrivetrainSubsystem, sArm, sTransfer, sShooter, kCenterHome, ShootingParameters.BELOW_SPEAKER);
        mChaseNoteCommand = new ChaseNoteCommand(drivetrainSubsystem, gamePieceProcessor, intake, transfer);
        

    }

    
    @Override
    public void initialize() {
        mCurrentState = AutoState.ZEROING_AND_SHOOT_PRELOAD;
        scheduleWithNextState(mZeroAndShootPreloadCommand);
    }

    /**
     * A wrapper for scheduling a command in auto that calls <code> determineNextState() </code> automatically when finished
     * @param currentCommand
     * @return
     */
    private Command scheduleWithNextState(Command currentCommand){
        return currentCommand.andThen(new InstantCommand(()->{
            determineNextState();
        }));
    }

    /**
     * Determine next state and command to run when a previous command is finished.
     * @return
     */
    public void determineNextState(){
        switch (mCurrentState) {
            case ZEROING_AND_SHOOT_PRELOAD:
                mCurrentState = AutoState.SEARCHING_NOTE;
                scheduleWithNextState(searchNextNote());
            case SEARCHING_NOTE:
                if(sGamePieceProcessor.getClosestGamePieceInfo().isPresent()){
                    mCurrentState = AutoState.CHASING_NOTE;
                    scheduleWithNextState(mChaseNoteCommand);
                } else {
                    scheduleWithNextState(searchNextNote());
                }
            case CHASING_NOTE:
                if(sTransfer.isOmronDetected()){
                    mCurrentState = AutoState.RETURN_AND_SHOOT;
                    scheduleWithNextState(returnAndShoot());
                }else{
                    mCurrentState = AutoState.SEARCHING_NOTE;
                    scheduleWithNextState(searchNextNote());
                }   
            case RETURN_AND_SHOOT: 
                // finished shooting, go to searching note
                mCurrentState = AutoState.SEARCHING_NOTE;
                scheduleWithNextState(searchNextNote());
                break;
            default:
                break;
        }
    }

    public Command returnAndShoot(){
        // TODO: implement path finding and following and shooting command
        // Section Based Return or path find return, and then shoot
        return buildPath("TODO ").until(()->!sTransfer.isOmronDetected());
    }

    public Command searchNextNote(){
        // TODO: implement path finding and following
        // sGamePieceProcessor.getNextOptimalNote();
        return buildPath("TODO").until(()->sGamePieceProcessor.getClosestGamePieceInfo().isPresent());
    }

    public Command buildPathFromPoint(GoalEndState goalEndState, Pose2d... poses) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

        return AutoBuilder.followPath(
            new PathPlannerPath(
                bezierPoints, new PathConstraints(3.75, 4.0, 3 * Math.PI, 4 * Math.PI), goalEndState));
    }

    private Command buildPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }


}
