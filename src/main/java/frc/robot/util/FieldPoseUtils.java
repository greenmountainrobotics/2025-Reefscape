package frc.robot.util;

import static frc.robot.constants.DriveConstants.WidthWithBumpersX;
import static frc.robot.constants.FieldConstants.*;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldPoseUtils {
  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(
        FieldWidth - pose.getX(),
        pose.getY(),
        pose.getRotation().plus(Rotation2d.fromDegrees(180)).times(-1));
  }

  public static Pose2d flipPoseIfRed(Pose2d pose) {
    if (MyAlliance.isRed()) return flipPose(pose);
    else return pose;
  }


  public static Supplier<Pose2d> flipPoseIfRed(Supplier<Pose2d> pose) {
    if (MyAlliance.isRed()) {
        return flipPose(pose, FieldWidth);
    } else {
        return pose;
    }
}

public static Supplier<Pose2d> flipPose(Supplier<Pose2d> poseSupplier, double fieldWidth) {
    return () -> {
        Pose2d pose = poseSupplier.get(); // Get the Pose2d from the Supplier
        return new Pose2d(
            fieldWidth - pose.getX(), // Flip the X-coordinate based on field width
            pose.getY(),             // Keep the Y-coordinate unchanged
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)) // Rotate by 180 degrees
        );
    };
}


  public static Translation2d flipTranslation(Translation2d translation) {
    return new Translation2d(FieldWidth - translation.getX(), translation.getY());
  }

  public static Translation2d flipTranslationIfRed(Translation2d translation) {
    if (MyAlliance.isRed()) return flipTranslation(translation);
    else return translation;
  }

  public static Pose2d alignedWithSourcePose() {
    Pose2d pose =
        new Pose2d(
            SourceCloseSideCorner.plus(SourceFarSideCorner)
                .div(2)
                .plus(new Translation2d(WidthWithBumpersX, 0).times(0.5).rotateBy(SourceRotation)),
            SourceRotation);
    if (MyAlliance.isRed()) pose = FieldPoseUtils.flipPose(pose);
    return pose;
  }

  public static Pose2d alignedWithAmpPose() {
    Pose2d pose =
        new Pose2d(
            AmpCenter.minus(
                new Translation2d(WidthWithBumpersX, 0)
                    .times(0.5)
                    .rotateBy(Rotation2d.fromDegrees(90))),
            AmpRotation);
    if (MyAlliance.isRed()) pose = FieldPoseUtils.flipPose(pose);
    return pose;
  }
}
