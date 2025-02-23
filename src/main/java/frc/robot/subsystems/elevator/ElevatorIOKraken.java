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

  private final DigitalInput limitSwitch = new DigitalInput(IdConstants.DIOId.LimitSwitchId);



  public ElevatorIOKraken() {

    TalonFXConfiguration configLeft = new TalonFXConfiguration();
    configLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configLeft.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimitAmps;
    configLeft.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXConfiguration configRight = new TalonFXConfiguration();
    configRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configRight.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimitAmps;
    configRight.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevatorLeft.getConfigurator().apply(configLeft);
    elevatorRight.getConfigurator().apply(configRight);

    //Set Leader follower 
    elevatorRight.setControl(new com.ctre.phoenix6.controls.Follower(IdConstants.CANId.LeftElevatorMotorId, true));

    elevatorLeft.setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPositionRads = elevatorLeft.getPosition();
    inputs.leftVelocityRadsPerSec = elevatorLeft.getVelocity();
    inputs.leftAppliedVoltage = elevatorLeft.getMotorVoltage();
    inputs.leftSupplyCurrentAmps = elevatorLeft.getSupplyCurrent();
    inputs.leftTorqueCurrentAmps = elevatorLeft.getTorqueCurrent();
    inputs.leftTempCelsius = elevatorLeft.getDeviceTemp();
    inputs.leftPositionTicks = elevatorLeft.getPosition().getValueAsDouble();


    inputs.rightPositionRads = elevatorRight.getPosition();
    inputs.rightVelocityRadsPerSec = elevatorRight.getVelocity();
    inputs.rightAppliedVoltage = elevatorRight.getMotorVoltage();
    inputs.rightSupplyCurrentAmps = elevatorRight.getSupplyCurrent();
    inputs.rightTorqueCurrentAmps = elevatorRight.getTorqueCurrent();
    inputs.rightTempCelsius = elevatorRight.getDeviceTemp();
    inputs.rightPositionTicks = elevatorLeft.getPosition().getValueAsDouble();



  }


  @Override
  public void runVoltage(double voltage) {
    elevatorLeft.setVoltage(voltage);
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
