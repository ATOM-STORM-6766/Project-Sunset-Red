package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceProcessor extends SubsystemBase {
    
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
