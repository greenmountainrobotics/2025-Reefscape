package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
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
  }

  public void setPosition(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;
    double pidOutput =
        elevatorPID.calculate(inputs.leftPositionTicks, targetPositionInches * ticksPerInch);
    io.runVoltage(12 * pidOutput);
  }

  public void goToLevelOne() {
    setPosition(levelOne);
  }

  public void goToLevelTwo() {
    setPosition(levelTwo);
  }

  public void goToLevelThree() {
    setPosition(levelThree);
  }

  public void goToLevelFour() {
    setPosition(levelFour);
  }

  public void goToGroundLevel() {
    setPosition(levelGroundInches);
  }

  public void goToBarge() {
    setPosition(levelBarge);
  }

  public void goToCoralPickup() {
    setPosition(levelPickup);
  }
}
