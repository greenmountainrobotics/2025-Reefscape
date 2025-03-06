package frc.robot.subsystems.climber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSIM implements ClimberIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final DCMotor leftExtentionMotor = DCMotor.DCMotor.getVex775Pro(1);
    private static final DCMotor rightExtentionMotor = DCMotorgetNEO(1);

    private DCMotorSim intakeSpinMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leftExtentionMotor, 1.0, 1.0), leftExtentionMotor);

    private DCMotorSim intakeRotateMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leftExtentionMotor, 1.0, 1.0), leftExtentionMotor);

    private double hangAppliedVolts = 0.0;
    private double leftCurrentAmps = 0.0;
    private double rightCurrentAmps = 0.0;

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        leftHangMotor.update(LOOP_PERIOD_SECS);
        rightHangMotor.update(LOOP_PERIOD_SECS);

        inputs.hangAppliedVolts = hangAppliedVolts;
        inputs.leftCurrentAmps = leftCurrentAmps;
        inputs.rightCurrentAmps = rightCurrentAmps;

        inputs.spinAppliedVolts = intakeSpinMotor.getInputVoltage();
        inputs.rightCurrentAmps = intakeSpinMotor.getCurrentDrawAmps();

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
