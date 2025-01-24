package frc.robot.constants;

public enum MyTrajectory {
  KnockOutMiddle("Knock out middle notes"),
  Taxi("Taxi");
  public final String fileName;

  MyTrajectory(String fileName) {
    this.fileName = fileName;
  }
}
