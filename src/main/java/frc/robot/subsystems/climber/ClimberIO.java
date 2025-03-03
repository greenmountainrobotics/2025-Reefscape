package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
<<<<<<< Updated upstream
    public double hangAppliedVolts = 0.0;
    public double currentDrawLeft = 0.0;
    public double currentDrawRight = 0.0;
    public double encoderRight = 0.0;
    public double encoderLeft = 0.0;
=======
    public double leftOutput;
    public double rightOutput;
>>>>>>> Stashed changes
  }

  default void updateInputs(ClimberIOInputs inputs) {}

<<<<<<< Updated upstream
  default void hangSetVoltage(double voltage) {}
=======
  default void setLeftVoltage(double voltage) {}

  default void setRightVoltage(double voltage) {}
>>>>>>> Stashed changes
}
