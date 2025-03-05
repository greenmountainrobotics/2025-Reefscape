package frc.robot.subsystems.endEffector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final DCMotor leftExtentionMotor = DCMotor.DCMotor.getVex775Pro(1);
    private static final DCMotor rightExtentionMotor = DCMotorgetNEO(1);

  private DCMotorSim intakeSpinMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(leftExtentionMotor, 1.0, 1.0), leftExtentionMotor);

  private DCMotorSim intakeRotateMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(leftExtentionMotor, 1.0, 1.0), leftExtentionMotor);


    private double spinAppliedVolts = 0.0;
    private double spinCurrentAmps = 0.0;

    private double articulationAppliedVolts = 0.0;
    private double articulationCurrentAmps = 0.0;

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        intakeSpinMotor.update(LOOP_PERIOD_SECS);
        intakeRotateMotor.update(LOOP_PERIOD_SECS);

        //inputs.articulationPosition  = intakeSpinMotor.();
        /*
        private double intakeSpinMotorVelocityRadPerSec = intakeSpinMotor.getAngularVelocityRadPerSec();
        private double intakeRotateMotorPositionRad = intakeRotateMotor.getAngularPositionRad();
        private double intakeRotateMotorVelocityRadPerSec = intakeRotateMotor.getAngularVelocityRadPerSec();
        */
        inputs.spinAppliedVolts = spinAppliedVolts;
        inputs.spinCurrentAmps = spinCurrentAmps;

        inputs.spinAppliedVolts = intakeSpinMotor.getInputVoltage();
        inputs.spinCurrentAmps = intakeSpinMotor.getCurrentDrawAmps();

        inputs.spinPositionRad = intakeSpinMotor.getAngularPositionRad();
        inputs.spinAngularVelocity = intakeSpinMotor.getAngularVelocity();

        inputs.articulationPositionRad = intakeRotateMotor.getAngularPositionRad();
        inputs.articulationAngularVelocity = intakeSpinMotor.getAngularVelocity();

}

    @Override
    public void articulationRunVoltage(double voltage) {
        articulationAppliedVolts = voltage;
        intakeRotateMotor.setInputVoltage(voltage); //why setINPUTVoltage?
    }

    @Override
    public void spinRunVoltage(double voltage) {
        spinAppliedVolts = voltage;
        intakeSpinMotor.setInputVoltage(voltage);  //why setINPUTVoltage?

    }
}

inputs.articulationAppliedVolts = intakeRotateMotor.getAppliedOutput();
inputs.articulationCurrentAmps = intakeRotateMotor.getOutputCurrent();


inputs.limitSwitchPressed = limitSwitch.get();
