package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
//import frc.robot.subsystems.leds.Leds;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagProvider[] implementations;

  public AprilTagVision(AprilTagProvider... implementations) {
    this.implementations = implementations;
  }

  @Override
  public void periodic() {
    ArrayList<Pose3d> targetPoses = new ArrayList<>();

    boolean isConnected = true;

    for (AprilTagProvider implementation : implementations) {
      implementation.periodic();
      targetPoses.addAll(Arrays.stream(implementation.getTargetPoses()).toList());
      if (!implementation.isConnected()) isConnected = false;
    }

 //   Leds.State.AprilTagsConnected = isConnected;

    Logger.recordOutput("AprilTagVision/TargetPoses", targetPoses.toArray(Pose3d[]::new));
  }

  public void setDataInterface(
      Drive.VisionMeasurementConsumer poseConsumer, Supplier<Pose2d> referencePoseSupplier) {
    for (AprilTagProvider implementation : implementations) {
      implementation.setDataInterface(poseConsumer, referencePoseSupplier);
    }
  }
}
