package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {

  private static final DCMotor leftExtentionMotor = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor rightExtentionMotor = DCMotor.getKrakenX60Foc(1);

  private DCMotorSim rightExtensionMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(leftExtentionMotor, 1.0, 1.0), leftExtentionMotor);

  private DCMotorSim leftExtensionMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(rightExtentionMotor, 1.0, 1.0), rightExtentionMotor);
  private static final double LOOP_PERIOD_SECS = 0.02;

  private double leftExtensionAppliedVoltage = 0.0;
  private double rightExtensionAppliedVoltage = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    leftExtensionMotor.update(LOOP_PERIOD_SECS);
    rightExtensionMotor.update(LOOP_PERIOD_SECS);

    inputs.rightPositionRads = rightExtensionMotor.getAngularPositionRad();
    inputs.rightVelocityRadsPerSec = rightExtensionMotor.getAngularVelocityRadPerSec();
    inputs.rightAppliedVoltage = rightExtensionAppliedVoltage;
    inputs.rightSupplyCurrentAmps = rightExtensionMotor.getCurrentDrawAmps();

    inputs.leftPositionRads = leftExtensionMotor.getAngularPositionRad();
    inputs.leftVelocityRadsPerSec = leftExtensionMotor.getAngularVelocityRadPerSec();
    inputs.leftAppliedVoltage = leftExtensionAppliedVoltage;
    inputs.rightSupplyCurrentAmps = leftExtensionMotor.getCurrentDrawAmps();
  }

  @Override
  public void runVoltage(double voltage) {
    leftExtensionAppliedVoltage = voltage;
    leftExtensionMotor.setInputVoltage(voltage);

    rightExtensionAppliedVoltage = voltage;
    rightExtensionMotor.setInputVoltage(voltage);
  }
}
