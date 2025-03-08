package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double leftHngAppliedVolts = 0.0;
    public double rightHangAppliedVolts = 0.0;
    public double currentDrawLeft = 0.0;
    public double currentDrawRight = 0.0;
    public double encoderRight = 0.0;
    public double encoderLeft = 0.0;
    public double leftOutput;
    public double rightOutput;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void hangSetVoltage(double voltage) {}
}
