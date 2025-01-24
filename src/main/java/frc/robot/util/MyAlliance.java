package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class Alliance {
  public static boolean isRed() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
