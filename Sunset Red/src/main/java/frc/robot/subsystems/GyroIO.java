package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

/*
 * Connects the software to the hardware and directly receives data from the gyroscope
 */
public interface GyroIO{

    /**
     * Reads informations from sources (hardware or simulation) and updates the inputs object
     * @param inputs
     */
    default void updateInputs(GyroIOInputs inputs){}
    default BaseStatusSignal[] getSignals(){   //! todo, check pheonix 6 for BaseStatusSignalValue[]
        return new BaseStatusSignal[0];
        
    }

    /**
     * Holds data that can be read from the corresponding gyroscope IO implementation
     */
    class GyroIOInputs{
        public boolean connected = false;
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
        public double angularVelocity = 0;
    }
}
