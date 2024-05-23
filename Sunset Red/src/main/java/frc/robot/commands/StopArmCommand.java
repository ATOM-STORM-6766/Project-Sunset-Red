package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class StopArmCommand extends InstantCommand {
    private final Arm mArm;

    public StopArmCommand(Arm arm) {
        mArm = arm;
        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mArm.stop();
    }
}
