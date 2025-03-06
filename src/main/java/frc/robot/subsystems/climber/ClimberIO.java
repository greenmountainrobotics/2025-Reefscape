package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double hangAppliedVolts = 0.0;

    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;

    public double encoderRight = 0.0;
    public double encoderLeft = 0.0;

    public double leftPositionRad = 0.0;
    public AngularVelocity leftAngularVelocity;


  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void hangRunVoltage(double voltage) {}
}
