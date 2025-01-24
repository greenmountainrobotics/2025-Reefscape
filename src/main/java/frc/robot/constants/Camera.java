package frc.robot.constants;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Function;

public enum Camera {
  BackCamera(
      "Front",
      1280,
      720,
      Rotation2d.fromDegrees(75),
      new Transform3d(
          new Translation3d(inchesToMeters(-2.091), inchesToMeters(-0.005), inchesToMeters(6.061)),
          new Rotation3d(0, degreesToRadians(-30), degreesToRadians(180))),
      (state) ->
          switch (state) {
            case NONE -> VecBuilder.fill(0.9, 0.9, 1.5);
            case ALIGNING_TO_SPEAKER, ALIGNING_TO_AMP -> VecBuilder.fill(0.9, 0.9, 0.9);
          }),
  FrontLeftCamera(
      "BackLeft",
      1280,
      720,
      Rotation2d.fromDegrees(70),
      new Transform3d(
          new Translation3d(inchesToMeters(6.894), inchesToMeters(11.382), inchesToMeters(10.621)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(60))),
      (state) ->
          switch (state) {
            case NONE -> VecBuilder.fill(0.9, 0.9, Double.POSITIVE_INFINITY);
            case ALIGNING_TO_SPEAKER, ALIGNING_TO_AMP -> VecBuilder.fill(
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
          }),
  FrontRightCamera(
      "BackRight",
      1280,
      720,
      Rotation2d.fromDegrees(70),
      new Transform3d(
          new Translation3d(inchesToMeters(6.894), inchesToMeters(-11.390), inchesToMeters(10.621)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(-60))),
      (state) ->
          switch (state) {
            case NONE -> VecBuilder.fill(0.9, 0.9, Double.POSITIVE_INFINITY);
            case ALIGNING_TO_SPEAKER, ALIGNING_TO_AMP -> VecBuilder.fill(
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
          });
  public final String name;
  public final Transform3d robotToCam;
  public final int width;
  public final int height;
  public final Rotation2d fov;
  public final Function<Drive.DriveState, Matrix<N3, N1>> stddev;

  Camera(
      String name,
      int width,
      int height,
      Rotation2d fov,
      Transform3d robotToCam,
      Function<Drive.DriveState, Matrix<N3, N1>> stddev) {
    this.name = name;
    this.robotToCam = robotToCam;
    this.width = width;
    this.height = height;
    this.fov = fov;
    this.stddev = stddev;
  }
}
