package frc.robot.subsystems.apriltagvision.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Camera;
import java.io.IOException;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonVisionIOSim implements PhotonVisionIO {
  private final PhotonCamera photonCamera;
  private final Camera camera;
  private final VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;

  public PhotonVisionIOSim(Camera camera, Supplier<Pose2d> poseSupplier) {
    photonCamera = new PhotonCamera(camera.name);
    this.camera = camera;
    this.poseSupplier = poseSupplier;

    visionSim = new VisionSystemSim("main");
    try {
      visionSim.addAprilTags(
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile));
    } catch (IOException error) {
      throw new Error(error);
    }

    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(camera.width, camera.height, camera.fov);

    PhotonCameraSim cameraSim = new PhotonCameraSim(photonCamera, cameraProp);

    visionSim.addCamera(cameraSim, camera.robotToCam);
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    inputs.isConnected = photonCamera.isConnected();
    inputs.latestResult = photonCamera.getLatestResult();
    inputs.timestamp = inputs.latestResult.getTimestampSeconds();
    inputs.camera = camera.name();
  }
}
