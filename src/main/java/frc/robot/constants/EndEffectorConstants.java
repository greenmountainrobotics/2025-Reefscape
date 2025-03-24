package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class EndEffectorConstants {
  // public static final double PivotHeight = inchesToMeters(8.369);
  // articulation is measured between extension and top side of end effector when retracted
  public static final Rotation2d CoralPlacementPosition = new Rotation2d(-20 * (Math.PI / 180));
  public static final Rotation2d CoralL4PlacementPosition = new Rotation2d(-10 * (Math.PI / 180));
  public static final Rotation2d CoralPickupRotation = new Rotation2d(45 * (Math.PI / 180));
  public static final Rotation2d BargePlacementRotation = new Rotation2d(68 * (Math.PI / 180));
  public static final Rotation2d TroftPlacementRotation = new Rotation2d(0 * (Math.PI / 180));

  /*public static final Rotation2d TargetArticulation = Rotation2d.fromDegrees(-77);
  public static final Rotation2d RetractedArticulation = Rotation2d.fromDegrees(90 + 36);
  public static final Rotation2d HalfwayArticulation = Rotation2d.fromDegrees(180 + 166.600512);
  public static final Rotation2d PointingAtShooterArticulation =
      Rotation2d.fromDegrees(180 - 52.993);
  public static final Rotation2d PointingUpArticulation = Rotation2d.fromDegrees(180 - 100.405);*/

  public static final double ArticulationToleranceRad = Math.toRadians(20);

  public static final double ArticulationVelocity = 0;

  public static final double voltageMultiplier = 0.5;

  public static final double ArticulationAcceleration = 0;

  public static final double IntakeSpeed = 1.0;
  public static final double PlacementSpeed = -1.0;

  public static final Rotation2d AbsoluteEncoderOffset = new Rotation2d(0.602 * 2 * Math.PI);
  // Offset is masured as the rotations when straight up
}
