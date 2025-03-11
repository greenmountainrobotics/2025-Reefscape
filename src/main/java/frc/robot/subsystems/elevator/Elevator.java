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
  private double gravityVolts = 0;
  public double targetPositionInches = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    elevatorPID =
        new PIDController(
            TunableConstants.KpElevator, TunableConstants.KiElevator, TunableConstants.KdElevator);
    elevatorPID.setTolerance(0.1);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/TargetInches", targetPositionInches);
    Logger.recordOutput(
        "Elevator/TargetRotations", targetPositionInches * MOTOR_ROTATIONS_PER_INCH);

    Logger.recordOutput("Elevator/leftPositionRads", inputs.leftPositionRots);
    Logger.recordOutput("Elevator/leftVelocityRadsPerSec", inputs.leftVelocityRadsPerSec);
    Logger.recordOutput("Elevator/leftAppliedVoltage", inputs.leftAppliedVoltage);
    Logger.recordOutput("Elevator/leftSupplyCurrentAmps", inputs.leftSupplyCurrentAmps);
    Logger.recordOutput("Elevator/leftTempCelsius", inputs.leftTempCelsius);
    Logger.recordOutput("Elevator/leftPositionTicks", inputs.leftPositionTicks);

    Logger.recordOutput("Elevator/rightPositionRads", inputs.rightPositionRots);
    Logger.recordOutput("Elevator/rightVelocityRadsPerSec", inputs.rightVelocityRadsPerSec);
    Logger.recordOutput("Elevator/rightAppliedVoltage", inputs.rightAppliedVoltage);
    Logger.recordOutput("Elevator/rightSupplyCurrentAmps", inputs.rightSupplyCurrentAmps);
    Logger.recordOutput("Elevator/rightTempCelsius", inputs.rightTempCelsius);
    Logger.recordOutput("Elevator/rightPositionTicks", inputs.rightPositionTicks);

    gravityVolts = 0.05;

    io.runVoltage(
        Math.max(
                -1.0,
                Math.min(
                    1.0,
                    (elevatorPID.calculate(
                            inputs.leftPositionTicks,
                            targetPositionInches * MOTOR_ROTATIONS_PER_INCH))
                        + gravityVolts))
            * 4);
  }

  public void setPosition(double pos) {
    targetPositionInches = pos;
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

  public InstantCommand goToMaxL1() {
    return new InstantCommand(() -> setPosition(maxL1));
  }

  public InstantCommand increase() {
    return new InstantCommand(() -> setPosition(targetPositionInches += 1.0));
  }

  public InstantCommand decrease() {
    return new InstantCommand(() -> setPosition(targetPositionInches -= 1.0));
  }
}
