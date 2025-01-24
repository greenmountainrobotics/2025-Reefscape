package frc.robot.subsystems.apriltagvision.photonvision;

import frc.robot.constants.Camera;
import org.photonvision.PhotonCamera;

public class PhotonVisionIOReal implements PhotonVisionIO {
  private final PhotonCamera photonCamera;
  private final Camera camera;

  public PhotonVisionIOReal(Camera camera) {
    photonCamera = new PhotonCamera(camera.name);
    this.camera = camera;
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.isConnected = photonCamera.isConnected();
    inputs.latestResult = photonCamera.getLatestResult();
    inputs.timestamp = inputs.latestResult.getTimestampSeconds();
    inputs.camera = camera.name();
  }
}
