package frc.robot.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

public final class DriveConstants {

  // Reef aliggnment constants
  public static final double ReefOffsetY = 7 * 0.0254; // 6.5 inches
  public static final double ReefOffsetX = -21 * 0.0254; // 20Inches

  public static final double CoralStationY = 9 * 0.0254; // 6.5 inches along the face
  public static final double CoralStationX = -15 * 0.0254; // 20Inches away from the face

  public static final double TrackWidthY = inchesToMeters(23.750);
  public static final double TrackWidthX = inchesToMeters(23.750);
  public static final double WidthWithBumpersY = inchesToMeters(30.875); // Not Updated
  public static final double WidthWithBumpersX = inchesToMeters(30.875); // Not Updated

  public static final double ReefPlacingDistance = 1.0;

  // Gear ratios for SDS MK4i L3
  public static final double DriveGearRatio =
      (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // Not Updated
  public static final double TurnGearRatio = 150.0 / 7.0; // Not Updated

  public static final double FrontLeftEncoderOffset = 0.022; // 1.718
  public static final double FrontRightEncoderOffset = 0.018; // -2.331
  public static final double BackLeftEncoderOffset = 0.019; // 2.477
  public static final double BackRightEncoderOffset = -0.029; // -0.933

  public static final double DriveTolerance = 0.075;
  public static final double ThetaToleranceRad = 0.03;

  public static final int kPigeonId = 13;
  public static final String DriveBaseName = "Drivetrain";
}
