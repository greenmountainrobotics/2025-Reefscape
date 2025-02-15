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
    public boolean leftMotorConnected = true;
    public double leftPositionRads = 0.0;
    public double leftVelocityRadsPerSec = 0.0;
    public double leftAppliedVoltage = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public boolean rightMotorConnected = true;
    public double rightPositionRads = 0.0;
    public double rightVelocityRadsPerSec = 0.0;
    public double rightAppliedVoltage = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runVoltage(double amps) {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}
