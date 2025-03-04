package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  // public static final double PivotHeight = inchesToMeters(8.369);
  // articulation is measured between extension and top side of end effector when retracted
  public static final Rotation2d DownRotation = new Rotation2d(0.165);
  public static final Rotation2d UpRotation = new Rotation2d(0.325);
  public static final double AbsoluteEncoderOffsetRads = 0.325;

  /*public static final Rotation2d TargetArticulation = Rotation2d.fromDegrees(-77);
  public static final Rotation2d RetractedArticulation = Rotation2d.fromDegrees(90 + 36);
  public static final Rotation2d HalfwayArticulation = Rotation2d.fromDegrees(180 + 166.600512);
  public static final Rotation2d PointingAtShooterArticulation =
      Rotation2d.fromDegrees(180 - 52.993);
  public static final Rotation2d PointingUpArticulation = Rotation2d.fromDegrees(180 - 100.405);*/

  public static final double ArticulationToleranceRad = 0.5;

  public static final double ArticulationVelocity = 0;
  public static final double ArticulationAcceleration = 0;

  public static final double IntakeSpeed = -0.30;
  public static final double TransferSpeed = -0.5;
}
