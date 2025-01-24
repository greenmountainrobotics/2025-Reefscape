package frc.robot.subsystems.apriltagvision.photonvision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
  @AutoLog
  class PhotonVisionIOInputs {
    public boolean isConnected;
    public PhotonPipelineResult latestResult = new PhotonPipelineResult();
    public double timestamp = -1.0;
    public String camera = "";
  }

  default void updateInputs(PhotonVisionIOInputs inputs) {}
}
