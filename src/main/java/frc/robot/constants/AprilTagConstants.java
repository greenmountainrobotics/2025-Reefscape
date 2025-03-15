package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagConstants {
  public static final Pose2d[] TAGS = {
    new Pose2d(new Translation2d(530.49, 130.17), new Rotation2d(Math.toRadians(300))), // id6
    new Pose2d(new Translation2d(530.49, 130.17), new Rotation2d(Math.toRadians(0))), // id7
    new Pose2d(new Translation2d(530.49, 186.83), new Rotation2d(Math.toRadians(60))), // id8
    new Pose2d(new Translation2d(497.77, 186.83), new Rotation2d(Math.toRadians(120))), // id9
    new Pose2d(new Translation2d(481.39, 158.50), new Rotation2d(Math.toRadians(180))), // id10
    new Pose2d(new Translation2d(497.77, 130.17), new Rotation2d(Math.toRadians(240))), // id11
    new Pose2d(new Translation2d(160.39, 130.17), new Rotation2d(Math.toRadians(240))), // id17
    new Pose2d(new Translation2d(144.00, 158.50), new Rotation2d(Math.toRadians(180))), // id18
    new Pose2d(new Translation2d(160.39, 186.83), new Rotation2d(Math.toRadians(120))), // id19
    new Pose2d(new Translation2d(193.10, 186.83), new Rotation2d(Math.toRadians(60))), // id20
    new Pose2d(new Translation2d(209.49, 158.50), new Rotation2d(Math.toRadians(0))), // id21
    new Pose2d(new Translation2d(193.10, 130.17), new Rotation2d(Math.toRadians(300))) // id22
  };
}
