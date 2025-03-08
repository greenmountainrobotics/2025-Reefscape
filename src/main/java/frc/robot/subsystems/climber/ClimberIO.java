package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
<<<<<<< HEAD
    public double leftHngAppliedVolts = 0.0;
    public double rightHangAppliedVolts = 0.0;
    public double currentDrawLeft = 0.0;
    public double currentDrawRight = 0.0;
=======
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;

    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;

>>>>>>> 342ace4141d35a54f6924bf820f4f386d0641201
    public double encoderRight = 0.0;
    public double encoderLeft = 0.0;

    public double leftPositionRad = 0.0;
    public AngularVelocity leftAngularVelocity;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void leftHangRunVoltage(double voltage) {}

  default void rightHangRunVoltage(double voltage) {}
}
