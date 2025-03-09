// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    // Status Signals
    public double leftPositionRots;
    public double leftVelocityRadsPerSec;
    public double leftAppliedVoltage;
    public double leftSupplyCurrentAmps;
    public double leftTorqueCurrentAmps;
    public double leftTempCelsius;
    public double leftPositionTicks;

    public double rightPositionRots;
    public double rightVelocityRadsPerSec;
    public double rightAppliedVoltage;
    public double rightSupplyCurrentAmps;
    public double rightTorqueCurrentAmps;
    public double rightTempCelsius;
    public double rightPositionTicks;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runVoltage(double volts) {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}
