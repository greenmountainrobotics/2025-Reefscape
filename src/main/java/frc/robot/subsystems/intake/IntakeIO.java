package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public Rotation2d articulationPosition = new Rotation2d();
    public Rotation2d articulationPositionReal = new Rotation2d();

    public double articulationAppliedVolts = 0.0;
    public double articulationCurrentAmps = 0.0;

    public double spinAppliedVolts = 0.0;
    public double spinCurrentAmps = 0.0;
    public boolean limitSwitchPressed = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void articulationRunVoltage(double voltage) {}

  default void spinRunVoltage(double voltage) {}
}
