package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final DCMotor leftExtentionMotor = DCMotor.getVex775Pro(1);
  private static final DCMotor rightExtentionMotor = DCMotor.getNEO(1);

  private DCMotorSim intakeSpinMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(leftExtentionMotor, 1.0, 1.0), leftExtentionMotor);

  private DCMotorSim intakeRotateMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(rightExtentionMotor, 1.0, 1.0), rightExtentionMotor);

  private double spinAppliedVolts = 0.0;
  private double spinCurrentAmps = 0.0;

  private double articulationAppliedVolts = 0.0;
  private double articulationCurrentAmps = 0.0;

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    intakeSpinMotorSim.update(LOOP_PERIOD_SECS);
    intakeRotateMotorSim.update(LOOP_PERIOD_SECS);

    inputs.spinAppliedVolts = spinAppliedVolts;
    inputs.spinCurrentAmps = spinCurrentAmps;

    inputs.articulationAppliedVolts = articulationAppliedVolts;
    inputs.articulationCurrentAmps = articulationCurrentAmps;

    inputs.spinAppliedVolts = intakeSpinMotorSim.getInputVoltage();
    inputs.spinCurrentAmps = intakeSpinMotorSim.getCurrentDrawAmps();

    inputs.spinPositionRad = intakeSpinMotorSim.getAngularPositionRad();
    inputs.spinAngularVelocity = intakeSpinMotorSim.getAngularVelocity();

    inputs.articulationPositionRad = intakeRotateMotorSim.getAngularPositionRad();
    inputs.articulationAngularVelocity = intakeSpinMotorSim.getAngularVelocity();

    // inputs.articulationAppliedVolts = intakeRotateMotor.getAppliedOutput();
    // inputs.articulationCurrentAmps = intakeRotateMotor.getOutputCurrent();

    // inputs.limitSwitchPressed = limitswitch.get();
  }

  @Override
  public void articulationRunVoltage(double voltage) {
    articulationAppliedVolts = voltage;
    intakeRotateMotorSim.setInputVoltage(voltage); // why setINPUTVoltage?
  }

  @Override
  public void spinRunVoltage(double voltage) {
    spinAppliedVolts = voltage;
    intakeSpinMotorSim.setInputVoltage(voltage); // why setINPUTVoltage?
  }
}
