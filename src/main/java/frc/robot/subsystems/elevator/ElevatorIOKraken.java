// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IdConstants;

public class ElevatorIOKraken implements ElevatorIO {
  // Hardware
  private final TalonFX elevatorLeft = new TalonFX(IdConstants.CANId.LeftElevatorMotorId);
  private final TalonFX elevatorRight = new TalonFX(IdConstants.CANId.RightElevatorMotorId);

  // private final RelativeEncoder rightEncoder;
  // private final RelativeEncoder leftEncoder;

  private final DigitalInput limitSwitch = new DigitalInput(IdConstants.DIOId.LimitSwitchId);

  // Status Signals
  private final StatusSignal<Angle> leftPositionRads;
  private final StatusSignal<AngularVelocity> leftVelocityRadsPerSec;
  private final StatusSignal<Voltage> leftAppliedVoltage;
  private final StatusSignal<Current> leftSupplyCurrentAmps;
  private final StatusSignal<Current> leftTorqueCurrentAmps;
  private final StatusSignal<Temperature> leftTempCelsius;

  private final StatusSignal<Angle> rightPositionRads;
  private final StatusSignal<AngularVelocity> rightVelocityRadsPerSec;
  private final StatusSignal<Voltage> rightAppliedVoltage;
  private final StatusSignal<Current> rightSupplyCurrentAmps;
  private final StatusSignal<Current> rightTorqueCurrentAmps;
  private final StatusSignal<Temperature> rightTempCelsius;

  public ElevatorIOKraken() {

    TalonFXConfiguration configLeft = new TalonFXConfiguration();
    configLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configLeft.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimitAmps;
    configLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorLeft.getConfigurator().apply(configLeft);

    TalonFXConfiguration configRight = new TalonFXConfiguration();
    configRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configRight.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimitAmps;
    configRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorRight.getConfigurator().apply(configRight);

    leftPositionRads = elevatorLeft.getPosition();
    leftVelocityRadsPerSec = elevatorLeft.getVelocity();
    leftAppliedVoltage = elevatorLeft.getMotorVoltage();
    leftSupplyCurrentAmps = elevatorLeft.getSupplyCurrent();
    leftTorqueCurrentAmps = elevatorLeft.getTorqueCurrent();
    leftTempCelsius = elevatorLeft.getDeviceTemp();

    rightPositionRads = elevatorRight.getPosition();
    rightVelocityRadsPerSec = elevatorRight.getVelocity();
    rightAppliedVoltage = elevatorRight.getMotorVoltage();
    rightSupplyCurrentAmps = elevatorRight.getSupplyCurrent();
    rightTorqueCurrentAmps = elevatorRight.getTorqueCurrent();
    rightTempCelsius = elevatorRight.getDeviceTemp();

    //  rightEncoder = elevatorRight.getEncoder();
  }

  // @Override
  // public void updateInputs(ElevatorIO inputs) {
  /*inputs.leftMotorConnected =
      BaseStatusSignal.refreshAll(
              leftPositionRads,
              leftVelocityRadsPerSec,
              leftAppliedVoltage,
              leftSupplyCurrentAmps,
              leftTorqueCurrentAmps,
              leftTempCelsius)
          .isOK();

  inputs.rightMotorConnected =
  BaseStatusSignal.refreshAll(
          rightPositionRads,
          rightVelocityRadsPerSec,
          rightAppliedVoltage,
          rightSupplyCurrentAmps,
          rightTorqueCurrentAmps,
          rightTempCelsius)
      .isOK();*/

  //  }

  @Override
  public void runVoltage(double volts) {
    elevatorLeft.setVoltage(volts);
  }

  @Override
  public void stop() {
    elevatorLeft.setVoltage(0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    elevatorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    elevatorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
