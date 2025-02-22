package frc.robot.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
 
    public static final double BranchTwoHeight = inchesToMeters(31.875);
    public static final double BranchThreeHeight = inchesToMeters(47.625);
    public static final double BranchFourHeight = inchesToMeters(72);
    //braches angles
    public static final double BranchAngle = 35; 
    public static final double BranchAngleFour = 90; //level 4 is Vertical 

    //Cage Info------------------------------------------------------------------------
    public static final double BottemHighCageHeightFromBarge = inchesToMeters(76);
    public static final double CageHeight = inchesToMeters(24);
    public static final double CageWidth = inchesToMeters(31.375);
    public static final Translation2d CageDimensions = 
        new Translation2d (CageWidth, CageHeight);
    public static final double ShallowCageHeightFromCarpet = inchesToMeters(30.125);
    //public static final double DeepCageHeightFromCarpet = inchesToMeters()

    public static final double CageOneDistanceFromMidFeild = inchesToMeters(41.5);
    public static final double CageTwoDistanceFromMidFeild = inchesToMeters(84.375);
    public static final double CageThreeDistanceFromMidFeild = inchesToMeters(127.375);
    //Barge Info-----------------------------------------------------------------------
    public static final double BargeDepth = inchesToMeters(46);
    public static final double BargeWith = inchesToMeters(146.5);
    public static final double BargeToReef = inchesToMeters(88);
    //Coral Station----------------------------------------------------------
    public static final double CoralMarkSize = inchesToMeters(4);
    public static final double CoralStationWith = inchesToMeters(70.875);
    //public static final double CoralStationDepth = inchesToMeters(164.375)

    j
}
