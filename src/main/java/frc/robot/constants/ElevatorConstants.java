package frc.robot.constants;

public final class ElevatorConstants {

  public static double currentLimitAmps = 40.0;

  public static final double GEAR_RATIO = 48.0 / 10.0;
  public static final double SPOOL_DIAMETER = 3.5;
  public static final double PI = Math.PI;

  public static final double MOTOR_ROTATIONS_PER_INCH = (GEAR_RATIO * 1.0) / (PI * SPOOL_DIAMETER);

  public static final double levelGroundInches = 0;

  public static final double maxL1 = 65.0;

  public static final double levelOne = 18.0;
  public static final double levelTwo = 31.875;
  public static final double levelThree = 47.625;
  public static final double levelFour = 72.0;
  public static final double levelBarge =
      96.0; // Assuming this is the height needed to interact with the Barge 96
  public static final double levelPickup =
      10.0; // Assuming this is the height for picking up game pieces
}

// First stage stroke = 21 inches
// Second stage stroke = 27
// third stage stroke = 28
