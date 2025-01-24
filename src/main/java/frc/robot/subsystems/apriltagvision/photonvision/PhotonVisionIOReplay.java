package frc.robot.subsystems.apriltagvision.photonvision;

import frc.robot.constants.Camera;

public class PhotonVisionIOReplay implements PhotonVisionIO {
  private final Camera camera;

  public PhotonVisionIOReplay(Camera camera) {
    this.camera = camera;
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.camera = camera.name();
  }
}
