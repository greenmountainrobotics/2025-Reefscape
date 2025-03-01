package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  class EndEffectorIOInputs {
    public Rotation2d articulationPosition = new Rotation2d();

    public double articulationAppliedVolts = 0.0;
    public double articulationCurrentAmps = 0.0;

    public double spinAppliedVolts = 0.0;
    public double spinCurrentAmps = 0.0;
    public boolean limitSwitchPressed = false;

    public double spinPositionRad = 0.0;
    public AngularVelocity spinAngularVelocity = 0.0;

    public AngularVelocity articulationPositionRad = 0.0;
    public double articulationAngularVelocity = 0.0;


  }

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void articulationRunVoltage(double voltage) {}

  default void spinRunVoltage(double voltage) {}
}
