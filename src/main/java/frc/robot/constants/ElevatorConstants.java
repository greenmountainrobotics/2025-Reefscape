package frc.robot.constants;

public final class ElevatorConstants {

  public static final double elevatorSpeed = 4; //Out of 12 
  public static double currentLimitAmps = 40.0;

  public static final double KpElevator = 0.1;
  public static final double KiElevator = 0;
  public static final double KdElevator = 0;

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

    public static final double firstStageStroke = 21.0;
    public static final double secondStageStroke = 27.0 + firstStageStroke;
    public static final double thirdStageStroke = 28.0 + secondStageStroke;

    public static final double firstStageVoltage = 0.02; 
    public static final double secondStageVoltage = 0.04;
    public static final double thirdStageVoltage = 0.06;

}

// First stage stroke = 21 inches
// Second stage stroke = 27
// third stage stroke = 28
