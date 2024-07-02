package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coprocessor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.utils.ShootingParameters;

public class AutoAimAndShootCommand extends Command {
    private final Shooter mShooter;
    private final Arm mArm;
    private final Transfer mTransfer;
    private final Coprocessor mCoprocessor;

    private ShootingParameters mShootingParameters;
    

    public AutoAimAndShootCommand(Shooter shooter, Arm arm, Transfer transfer, Coprocessor coprocessor){
        mShooter = shooter;
        mArm = arm;
        mTransfer = transfer;
        mCoprocessor = coprocessor;
        addRequirements(mShooter);
        addRequirements(mTransfer);
        addRequirements(mArm);
        // do not need to requre Coprocessor
    }

    @Override 
    public void initialize(){
        mShootingParameters = getParametersFromVision().get();
    }

    private Optional<ShootingParameters> getParametersFromVision(){
        return Optional.empty(); // TODO;
    }

    



}
