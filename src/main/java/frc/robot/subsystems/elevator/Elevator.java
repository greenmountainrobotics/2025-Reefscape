package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TunableConstants;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private final PIDController elevatorPID;

  private double extensionSetpointMeters = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    elevatorPID =
        new PIDController(
            TunableConstants.KpElevator, TunableConstants.KiElevator, TunableConstants.KdElevator);

    elevatorPID.setSetpoint(extensionSetpointMeters);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
