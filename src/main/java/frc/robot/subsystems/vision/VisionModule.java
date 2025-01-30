package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionModule implements VisionModuleIO {
    PhotonCamera camera;

    public VisionModule(String photonCameraID) {
        camera = new PhotonCamera(photonCameraID);
    }
}
