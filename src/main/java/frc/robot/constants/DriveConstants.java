package frc.robot.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

public final class DriveConstants {
  public static final double TrackWidthY = inchesToMeters(17.587);
  public static final double TrackWidthX = inchesToMeters(17.587);
  public static final double WidthWithBumpersY = inchesToMeters(30.875);
  public static final double WidthWithBumpersX = inchesToMeters(30.875);

  // Gear ratios for SDS MK4i L3
  public static final double DriveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double TurnGearRatio = 150.0 / 7.0;

  public static final double FrontLeftEncoderOffset = 0.022; // 1.718
  public static final double FrontRightEncoderOffset = 0.018; // -2.331
  public static final double BackLeftEncoderOffset = 0.019; // 2.477
  public static final double BackRightEncoderOffset = -0.029; // -0.933

  public static final double DriveTolerance = 0.025;
  public static final double ThetaToleranceRad = 0.03;
}
