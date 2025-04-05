package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;
  private final PIDController elevatorPID;

  private double driveWeight = 1.0;
  private double gravityVolts = 0;
  public double targetPositionInches = 0;

  public Elevator(ElevatorIO io) {

    this.io = io;
    elevatorPID =
        new PIDController(
            ElevatorConstants.KpElevator,
            ElevatorConstants.KiElevator,
            ElevatorConstants.KdElevator);
    elevatorPID.setTolerance(0.01);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/TargetInches", targetPositionInches);
    Logger.recordOutput(
        "Elevator/TargetRotations", targetPositionInches * MOTOR_ROTATIONS_PER_INCH);
    Logger.recordOutput("Elevator/gravityVolts", gravityVolts);

    Logger.recordOutput(
        "Elevator/positionInches", inputs.leftPositionRots * MOTOR_ROTATIONS_PER_INCH);

    Logger.recordOutput("Elevator/atTargetPosition", atTargetPosition());

    Logger.recordOutput("Elevator/leftPositionRots", inputs.leftPositionRots);
    Logger.recordOutput("Elevator/leftVelocityRadsPerSec", inputs.leftVelocityRadsPerSec);
    Logger.recordOutput("Elevator/leftAppliedVoltage", inputs.leftAppliedVoltage);
    Logger.recordOutput("Elevator/leftSupplyCurrentAmps", inputs.leftSupplyCurrentAmps);
    Logger.recordOutput("Elevator/leftTempCelsius", inputs.leftTempCelsius);
    Logger.recordOutput("Elevator/leftPositionTicks", inputs.leftPositionTicks);

    Logger.recordOutput("Elevator/rightPositionRots", inputs.rightPositionRots);
    Logger.recordOutput("Elevator/rightVelocityRadsPerSec", inputs.rightVelocityRadsPerSec);
    Logger.recordOutput("Elevator/rightAppliedVoltage", inputs.rightAppliedVoltage);
    Logger.recordOutput("Elevator/rightSupplyCurrentAmps", inputs.rightSupplyCurrentAmps);
    Logger.recordOutput("Elevator/rightTempCelsius", inputs.rightTempCelsius);
    Logger.recordOutput("Elevator/rightPositionTicks", inputs.rightPositionTicks);

    // Gravity compensation
    if (inputs.leftPositionTicks > 0
        && inputs.leftPositionTicks < ElevatorConstants.firstStageStroke) {
      gravityVolts = ElevatorConstants.firstStageVoltage;
    } else if (inputs.leftPositionTicks > ElevatorConstants.firstStageStroke
        && inputs.leftPositionTicks < ElevatorConstants.secondStageStroke) {
      gravityVolts = ElevatorConstants.secondStageVoltage;
    } else if (inputs.leftPositionTicks > ElevatorConstants.secondStageStroke) {
      gravityVolts = ElevatorConstants.thirdStageVoltage;
    }

    // io.runVoltage(12 * gravityVolts);
    io.runVoltage(
        Math.max(
                -1.0,
                Math.min(
                    1.0,
                    (ElevatorConstants.voltageMultiplier
                            * elevatorPID.calculate(inputs.leftPositionTicks, targetPositionInches))
                        + gravityVolts))
            * ElevatorConstants.elevatorSpeed);
    //  io.runVoltage(0);
  }

  public boolean atTargetPosition() {
    return Math.abs(inputs.leftPositionRots - (targetPositionInches)) < 0.5;
  }

  public boolean atTargetPositionL4() {
    return Math.abs(inputs.leftPositionRots - (targetPositionInches)) < 0.3;
  }

  public void setPosition(double pos) {
    targetPositionInches = pos;
    if (pos == levelFour) {
      driveWeight = 0.5;
    } else {
      driveWeight = 1.0;
    }
  }

  public double getDriveWeight() {
    return driveWeight;
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
