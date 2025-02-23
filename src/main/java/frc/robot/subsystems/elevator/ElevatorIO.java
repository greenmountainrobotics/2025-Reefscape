// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
  // Status Signals
  public StatusSignal<Angle> leftPositionRads;
  public StatusSignal<AngularVelocity> leftVelocityRadsPerSec;
  public StatusSignal<Voltage> leftAppliedVoltage;
  public StatusSignal<Current> leftSupplyCurrentAmps;
  public StatusSignal<Current> leftTorqueCurrentAmps;
  public StatusSignal<Temperature> leftTempCelsius;
  public double leftPositionTicks; 

  public StatusSignal<Angle> rightPositionRads;
  public StatusSignal<AngularVelocity> rightVelocityRadsPerSec;
  public StatusSignal<Voltage> rightAppliedVoltage;
  public StatusSignal<Current> rightSupplyCurrentAmps;
  public StatusSignal<Current> rightTorqueCurrentAmps;
  public StatusSignal<Temperature> rightTempCelsius;
  public double rightPositionTicks; 
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runVoltage(double volts) {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}
