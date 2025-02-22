package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Config;

public enum RunMode {
  /** Running on a real robot. */
  REAL,

  /** Running a physics simulator. */
  SIM,

  /** Replaying from a log file. */
  REPLAY;

  public static RunMode getMode() {
    if (RobotBase.isReal()) return RunMode.REAL;
    return Config.REPLAY ? RunMode.REPLAY : RunMode.SIM;
  }
}
