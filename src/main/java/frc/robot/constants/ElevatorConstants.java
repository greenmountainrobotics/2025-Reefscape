package frc.robot.constants;

public final class ElevatorConstants {

  public static final double elevatorSpeed = 4; // Out of 12
  public static double currentLimitAmps = 40.0;

  public static final double KpElevator = 0.40;
  public static final double KiElevator = 0.02;
  public static final double KdElevator = 0.055;
  public static final double voltageMultiplier = 0.5;

  public static final double GEAR_RATIO = 48.0 / 10.0;
  public static final double SPOOL_DIAMETER = 3.5;
  public static final double PI = Math.PI;

  public static final double MOTOR_ROTATIONS_PER_INCH = (GEAR_RATIO * 1.0) / (PI * SPOOL_DIAMETER);

  public static final double levelGroundInches = 0;

  public static final double maxL1 = 65.0; // 8.3

  public static final double levelOne = 7.5 - 1.1;
  public static final double levelTwo = 13.13 - 1.1;
  public static final double levelThree = 19.40 - 1.1;
  public static final double levelFour = 28.2;
  public static final double levelBarge = 28.2;
  public static final double levelPickup = 7.3;

  public static final double firstStageStroke = 9.138;
  public static final double secondStageStroke = 19.350;
  public static final double thirdStageStroke = 30.0; // 29.35

  public static final double firstStageVoltage = 0.020000;
  public static final double secondStageVoltage = 0.038000;
  public static final double thirdStageVoltage = 0.10000;
}

// First stage stroke = 21 inches
// Second stage stroke = 27
// third stage stroke = 28
