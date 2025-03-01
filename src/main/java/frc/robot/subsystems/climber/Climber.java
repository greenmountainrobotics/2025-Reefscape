<<<<<<< Updated upstream
package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  // public final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
=======
// Add pid controler
// 2 rotational motors
// leader follower
// spinnign opp directions


package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
>>>>>>> Stashed changes

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
<<<<<<< Updated upstream
    // io.updateInputs(inputs);
  }

  public void setSpeed(double speed) {
    io.hangSetVoltage(speed * 12.0);
  }
}
=======
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setVoltage(double voltage) {

    setVoltage(voltage, voltage);
  }

  public void setVoltage(double left, double right) {
    io.setLeftVoltage(left);
    io.setRightVoltage(right);
  }
}
>>>>>>> Stashed changes
