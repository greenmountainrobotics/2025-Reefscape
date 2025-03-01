<<<<<<< Updated upstream
package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TunableConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
=======
// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. jhgjh

package frc.robot.subsystems.elevator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;



public abstract class Elevator<G extends Elevator.SlamElevatorGoal> {

  public interface SlamElevatorGoal {
    DoubleSupplier getSlammingCurrent();

    boolean isStopAtGoal();

    default boolean isNonSensing() {
      return false;
    }

    SlamElevatorState getState();
  }

  public enum SlamElevatorState {
    IDLING,
    RETRACTING,
    EXTENDING
  }

>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
    Logger.recordOutput("Elevator/rightPositionRads", inputs.rightPositionRads);
    Logger.recordOutput("Elevator/rightVelocityRadsPerSec", inputs.rightVelocityRadsPerSec);
    Logger.recordOutput("Elevator/rightAppliedVoltage", inputs.rightAppliedVoltage);
    Logger.recordOutput("Elevator/rightSupplyCurrentAmps", inputs.rightSupplyCurrentAmps);
    Logger.recordOutput("Elevator/rightTempCelsius", inputs.rightTempCelsius);
    Logger.recordOutput("Elevator/rightPositionTicks", inputs.rightPositionTicks);
=======
    // Reset if changing goals
    if (lastGoal != null && getGoal() != lastGoal) {
      atGoal = false;
      staticTimer.stop();
      staticTimer.reset();
    }
    // Set last goal
    lastGoal = getGoal();

    // Set alert
    // disconnected.set(!inputs.motorConnected);

    // Check if at goal.
    
    if (!atGoal) {
      // Start static timer if within min velocity threshold.
      if (getGoal().isNonSensing() || Math.abs(inputs.velocityRadsPerSec) <= minVelocityThresh) {
        staticTimer.start();
      } else {
        staticTimer.stop();
        staticTimer.reset();
      }
      // If we are finished with timer, finish goal.
      // Also assume we are at the goal if auto was started
      atGoal = staticTimer.hasElapsed(staticTimeSecs) || DriverStation.isAutonomousEnabled();
    } else {
      staticTimer.stop();
      staticTimer.reset();
    }

    // Run to goal.
    if (!atGoal) {
      io.runCurrent(getGoal().getSlammingCurrent().getAsDouble());
    } else {
      if (getGoal().isStopAtGoal()) {
        io.stop();
      } else {
        io.runCurrent(getGoal().getSlammingCurrent().getAsDouble());
      }
    }

    if (DriverStation.isDisabled()) {
      // Reset
      io.stop();
      lastGoal = null;
      staticTimer.stop();
      staticTimer.reset();
      if (Math.abs(inputs.velocityRadsPerSec) > minVelocityThresh) {
        // If we don't move when disabled, assume we are still at goal
        atGoal = false;
      }
    }

    // Update coast mode
    setBrakeMode(!coastModeSupplier.getAsBoolean());

    Logger.recordOutput("Superstructure/" + name + "/Goal", getGoal().toString());
>>>>>>> Stashed changes
  }

  public void setPosition(double targetPositionInches) {
    this.targetPositionInches = targetPositionInches;
    double pidOutput =
        elevatorPID.calculate(inputs.leftPositionTicks, targetPositionInches * ticksPerInch);
    io.runVoltage(12 * pidOutput);
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
