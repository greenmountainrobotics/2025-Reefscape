package frc.robot.constants;

public enum Trajectory {
  KnockOutMiddle("Knock out middle notes"),
  Taxi("Taxi");
  public final String fileName;

  Trajectory(String fileName) {
    this.fileName = fileName;
  }
}
