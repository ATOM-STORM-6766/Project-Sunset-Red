package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib6907.DelayedBoolean;

public class GamePieceProcessor extends SubsystemBase {

    public static class Target{
        public double pitch; // in pixel
        public double yaw; // in pixel
    }
    
    private static GamePieceProcessor mCoprocessor = new GamePieceProcessor();
    private PhotonCamera mUSBCamera = new PhotonCamera("USBCamera");
    

    public static GamePieceProcessor getInstance() {
        return mCoprocessor;
    }

    public GamePieceProcessor(){
    }

    public Optional<PhotonTrackedTarget> getClosestGamePieceInfo(){
        if(mUSBCamera.getLatestResult().hasTargets()){
            return Optional.of(mUSBCamera.getLatestResult().getBestTarget());
        }
        return Optional.empty();
    }
}
