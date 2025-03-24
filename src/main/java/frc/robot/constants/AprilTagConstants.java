package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagConstants {
  public static final Pose2d[] TAGS = {
    new Pose2d(
        new Translation2d(530.49 * 0.0254, 130.17 * 0.0254),
        new Rotation2d(Math.toRadians(300))), // id6
    new Pose2d(
        new Translation2d(530.49 * 0.0254, 130.17 * 0.0254),
        new Rotation2d(Math.toRadians(0))), // id7
    new Pose2d(
        new Translation2d(530.49 * 0.0254, 186.83 * 0.0254),
        new Rotation2d(Math.toRadians(60))), // id8
    new Pose2d(
        new Translation2d(497.77 * 0.0254, 186.83 * 0.0254),
        new Rotation2d(Math.toRadians(120))), // id9
    new Pose2d(
        new Translation2d(481.39 * 0.0254, 158.50 * 0.0254),
        new Rotation2d(Math.toRadians(180))), // id10
    new Pose2d(
        new Translation2d(497.77 * 0.0254, 130.17 * 0.0254),
        new Rotation2d(Math.toRadians(240))), // id11
    new Pose2d(
        new Translation2d(160.39 * 0.0254, 130.17 * 0.0254),
        new Rotation2d(Math.toRadians(240))), // id17
    new Pose2d(
        new Translation2d(144.00 * 0.0254, 158.50 * 0.0254),
        new Rotation2d(Math.toRadians(180))), // id18
    new Pose2d(
        new Translation2d(160.39 * 0.0254, 186.83 * 0.0254),
        new Rotation2d(Math.toRadians(120))), // id19
    new Pose2d(
        new Translation2d(193.10 * 0.0254, 186.83 * 0.0254),
        new Rotation2d(Math.toRadians(60))), // id20
    new Pose2d(
        new Translation2d(209.49 * 0.0254, 158.50 * 0.0254),
        new Rotation2d(Math.toRadians(0))), // id21
    new Pose2d(
        new Translation2d(193.10 * 0.0254, 130.17 * 0.0254),
        new Rotation2d(Math.toRadians(300))) // id22
  };

  //1, 2, 12, 13
  public static final Pose2d[] CORALPICKUPTAGS = {
    new Pose2d(
        new Translation2d(657.37 * 0.0254, 25.80 * 0.0254),
        new Rotation2d(Math.toRadians(126))), // id1

    new Pose2d(
        new Translation2d(657.37 * 0.0254, 291.20 * 0.0254),
        new Rotation2d(Math.toRadians(234))), // id2

    new Pose2d(
        new Translation2d(33.51 * 0.0254, 25.80 * 0.0254),
        new Rotation2d(Math.toRadians(54))), // id12

    new Pose2d(
        new Translation2d(33.51 * 0.0254, 291.20 * 0.0254),
        new Rotation2d(Math.toRadians(306))) // id13
  };
}
