package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class EndEffectorConstants {
  // public static final double PivotHeight = inchesToMeters(8.369);
  // articulation is measured between extension and top side of end effector when retracted
  public static final Rotation2d DownRotation = Rotation2d.fromDegrees(-77);
  public static final Rotation2d UpRotation = Rotation2d.fromDegrees(0);

  /*public static final Rotation2d TargetArticulation = Rotation2d.fromDegrees(-77);
  public static final Rotation2d RetractedArticulation = Rotation2d.fromDegrees(90 + 36);
  public static final Rotation2d HalfwayArticulation = Rotation2d.fromDegrees(180 + 166.600512);
  public static final Rotation2d PointingAtShooterArticulation =
      Rotation2d.fromDegrees(180 - 52.993);
  public static final Rotation2d PointingUpArticulation = Rotation2d.fromDegrees(180 - 100.405);*/

  public static final double ArticulationToleranceRad = 0.5;

  public static final double ArticulationVelocity = 0;
  public static final double ArticulationAcceleration = 0;

  public static final double IntakeSpeed = -0.5;
  public static final double PlacementSpeed = -0.5;

  public static final Rotation2d AbsoluteEncoderOffset =
      Rotation2d.fromRadians(-1.635).unaryMinus();
}
