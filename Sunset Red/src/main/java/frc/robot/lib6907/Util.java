package frc.robot.lib6907;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {
    public static final double DEADBAND = 0.05;

    public static Rotation2d nearestPole(Rotation2d rot) {
        double pole_sin = 0.0;
    	double pole_cos = 0.0;
    	if(Math.abs(rot.getCos()) > Math.abs(rot.getSin())){
    		pole_cos = Math.signum(rot.getCos());
    		pole_sin = 0.0;
    	}else{
    		pole_cos = 0.0;
    		pole_sin = Math.signum(rot.getSin());
    	}
    	return new Rotation2d(pole_cos, pole_sin);
    }

    public static double eliminateDeadband(double input) {
        if (Math.abs(input) < DEADBAND)
            return 0.0;
        return input;
    }
    
}
