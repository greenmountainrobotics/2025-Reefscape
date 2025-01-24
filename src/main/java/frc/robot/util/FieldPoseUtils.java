package frc.robot.util;

import static frc.robot.constants.DriveConstants.WidthWithBumpersX;
import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.FieldConstants.SourceRotation;

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
    if (Alliance.isRed()) return flipPose(pose);
    else return pose;
  }

  public static Translation2d flipTranslation(Translation2d translation) {
    return new Translation2d(FieldWidth - translation.getX(), translation.getY());
  }

  public static Translation2d flipTranslationIfRed(Translation2d translation) {
    if (Alliance.isRed()) return flipTranslation(translation);
    else return translation;
  }

  public static Pose2d alignedWithSourcePose() {
    Pose2d pose =
        new Pose2d(
            SourceCloseSideCorner.plus(SourceFarSideCorner)
                .div(2)
                .plus(new Translation2d(WidthWithBumpersX, 0).times(0.5).rotateBy(SourceRotation)),
            SourceRotation);
    if (Alliance.isRed()) pose = FieldPoseUtils.flipPose(pose);
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
    if (Alliance.isRed()) pose = FieldPoseUtils.flipPose(pose);
    return pose;
  }
}
