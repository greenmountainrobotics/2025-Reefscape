package frc.robot.constants;

public final class IdConstants {
  public static final class CANId {
    // This might end up being canivore

    public static final int LeftElevatorMotorId = 22; // Kwaken
    public static final int RightElevatorMotorId = 23; // Kwaken

    public static final int IntakeRotateMotorId = 16; // 775 (Spark Max Brushed)
    public static final int IntakeSpinMotorId = 17; // Neo Vortex (Spark Flex)

    public static final int EndEffectorRotateMotorId = 18; // Neo (Spark Max Brushless)
    public static final int EndEffectorSpinMotorId = 19; // 775 (Spark Max Brushed)

    public static final int LeftHangMotor = 20;
    public static final int RightHangMotor = 21;
  }

  public static final class DIOId {
    public static final int LimitSwitchId = 2;
    public static final int IntakeArticulationEncoderId = 3;
  }
}
