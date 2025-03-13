package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private double Speed;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);

    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/Speed", Speed);
  }

  public Command setSpeed(double speed) {
    Speed = speed;
    return new InstantCommand(() -> setClimb(speed));
  }

  public void setClimb(double speed) {
    io.setClimbVoltage(12.0 * speed);
  }
}
