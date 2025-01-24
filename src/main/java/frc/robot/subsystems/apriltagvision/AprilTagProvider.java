package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drive.DriveU;
import java.util.function.Supplier;

public interface AprilTagProvider {
  void setDataInterface(
      DriveU.VisionMeasurementConsumer poseConsumer, Supplier<Pose2d> referencePoseSupplier);

  void periodic();

  Pose3d[] getTargetPoses();

  boolean isConnected();
}
