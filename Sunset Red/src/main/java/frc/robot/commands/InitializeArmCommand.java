package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class InitializeArmCommand extends Command {
    private final Arm mArm;
    private static final double ARM_CLIMB_VOLTS = -2.0;
    private static final double INIT_CURRENT_THRESHOLD = 30.0;
    public InitializeArmCommand(Arm arm) {
        mArm = arm;
        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mArm.setReverseLimit(false);
        mArm.setVoltage(ARM_CLIMB_VOLTS);
    }

    @Override
    public void execute() {
        if (mArm.getArmCurrent() < -INIT_CURRENT_THRESHOLD) {
            mArm.setAngle(ArmConstants.ARM_REST_POSITION);
            mArm.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mArm.stop();
    }

    @Override
    public boolean isFinished() {
        return mArm.getInPosition();
    }
}
