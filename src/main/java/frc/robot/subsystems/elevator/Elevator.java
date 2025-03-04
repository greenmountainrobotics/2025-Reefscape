package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TunableConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private final PIDController elevatorPID;
  private double targetPositionInches = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    elevatorPID =
        new PIDController(
            TunableConstants.KpElevator, TunableConstants.KiElevator, TunableConstants.KdElevator);
    elevatorPID.setTolerance(100);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/TargetInches", targetPositionInches);
    Logger.recordOutput("Elevator/leftPositionRads", inputs.leftPositionRads);
    Logger.recordOutput("Elevator/leftVelocityRadsPerSec", inputs.leftVelocityRadsPerSec);
    Logger.recordOutput("Elevator/leftAppliedVoltage", inputs.leftAppliedVoltage);
    Logger.recordOutput("Elevator/leftSupplyCurrentAmps", inputs.leftSupplyCurrentAmps);
    Logger.recordOutput("Elevator/leftTempCelsius", inputs.leftTempCelsius);
    Logger.recordOutput("Elevator/leftPositionTicks", inputs.leftPositionTicks);

    Logger.recordOutput("Elevator/rightPositionRads", inputs.rightPositionRads);
    Logger.recordOutput("Elevator/rightVelocityRadsPerSec", inputs.rightVelocityRadsPerSec);
    Logger.recordOutput("Elevator/rightAppliedVoltage", inputs.rightAppliedVoltage);
    Logger.recordOutput("Elevator/rightSupplyCurrentAmps", inputs.rightSupplyCurrentAmps);
    Logger.recordOutput("Elevator/rightTempCelsius", inputs.rightTempCelsius);
    Logger.recordOutput("Elevator/rightPositionTicks", inputs.rightPositionTicks);

    io.runVoltage(
        12 * elevatorPID.calculate(inputs.leftPositionTicks, targetPositionInches * ticksPerInch));
  }

  public void setPosition(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;
  }

  public InstantCommand goToLevelOne() {
    return new InstantCommand(() -> setPosition(levelOne));
  }

  public InstantCommand goToLevelTwo() {
    return new InstantCommand(() -> setPosition(levelTwo));
  }

  public InstantCommand goToLevelThree() {
    return new InstantCommand(() -> setPosition(levelThree));
  }

  public InstantCommand goToLevelFour() {
    return new InstantCommand(() -> setPosition(levelFour));
  }

  public InstantCommand goToGroundLevel() {
    return new InstantCommand(() -> setPosition(levelGroundInches));
  }

  public InstantCommand goToBarge() {
    return new InstantCommand(() -> setPosition(levelBarge));
  }

  public InstantCommand goToCoralPickup() {
    return new InstantCommand(() -> setPosition(levelPickup));
  }
}
