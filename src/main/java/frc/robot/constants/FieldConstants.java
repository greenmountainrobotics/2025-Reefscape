package frc.robot.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
  public static final double FieldWidth =
      inchesToMeters(
          76.1 * 2 // 2 * width of amp
              + 250.50 * 2); // 2 * width between centerline and source
  public static final double FieldHeight = inchesToMeters(323.00);

  public static final Translation2d AmpCenter =
      new Translation2d(inchesToMeters(72.5), FieldHeight);

  // facing out
  public static final Rotation2d AmpRotation = Rotation2d.fromDegrees(-90);

  public static final Translation2d SourceCloseSideCorner =
      new Translation2d(
          FieldWidth - inchesToMeters(76.1), // field width - width of amp
          0.0 // on bottom edge of field
          );
  public static final Translation2d SourceFarSideCorner =
      new Translation2d(
          FieldWidth, inchesToMeters(60.75 - 18.0)); // height from edge - height of wall

  // facing out
  public static final Rotation2d SourceRotation = Rotation2d.fromDegrees(120);

  public static final Translation2d SpeakerBottomCloseSideCorner =
      new Translation2d(0, SourceFarSideCorner.getY() + inchesToMeters(133.677));

  public static final Translation2d SpeakerTopCloseSideCorner =
      new Translation2d(0, SpeakerBottomCloseSideCorner.getY() + inchesToMeters(82.400));

  private static final Translation2d SpeakerCloseCornerToFarCorner =
      new Translation2d(inchesToMeters(35.694542), inchesToMeters(20.824760));

  public static final Translation2d SpeakerBottomFarSideCorner =
      SpeakerBottomCloseSideCorner.plus(SpeakerCloseCornerToFarCorner);

  public static final Translation2d SpeakerTopFarSideCorner =
      SpeakerTopCloseSideCorner.plus(
          new Translation2d(
              SpeakerCloseCornerToFarCorner.getX(), -SpeakerCloseCornerToFarCorner.getY()));

  public static final Translation2d SpeakerCloseSideCenter =
      SpeakerBottomCloseSideCorner.plus(SpeakerTopCloseSideCorner).div(2);

  public static final Rotation2d SpeakerRotation = new Rotation2d();

  public static final double SpeakerShootingDistance = 2.05;

  public static final double InnerNoteDistanceFromCloseSide = inchesToMeters(114);
  public static final double DistanceBetweenInnerNotes = inchesToMeters(57);

  public static final Translation2d BottomInnerNote =
      new Translation2d(InnerNoteDistanceFromCloseSide, FieldHeight / 2);
  public static final Translation2d MiddleInnerNote =
      BottomInnerNote.plus(new Translation2d(0, DistanceBetweenInnerNotes));
  public static final Translation2d TopInnerNote =
      MiddleInnerNote.plus(new Translation2d(0, DistanceBetweenInnerNotes));

  public static final double NoteDiameter = inchesToMeters(14);
}
