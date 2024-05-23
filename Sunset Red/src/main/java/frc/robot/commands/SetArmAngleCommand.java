package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmAngleCommand extends Command{
    private final Arm sArm;
    private final double targetAngle;

    public SetArmAngleCommand(Arm arm, double angle){
        sArm = arm;
        targetAngle = angle;
        addRequirements(sArm);
    }

    @Override
    public void initialize(){
        sArm.setAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted){
        sArm.stop();
    }

    @Override
    public boolean isFinished(){
        return sArm.getInPosition();
    }
}
