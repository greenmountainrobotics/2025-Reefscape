package frc.robot.subsystems.climber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSIM implements ClimberIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final DCMotor leftExtentionMotor = DCMotor.DCMotor.getVex775Pro(1);
    private static final DCMotor rightExtentionMotor = DCMotor.DCMotor.getNEO(1);

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

        inputs.hangAppliedVolts = leftExtentionMotor.getInputVoltage();

        inputs.rightCurrentAmps = rightExtentionMotor.getCurrentDrawAmps();
        inputs.leftCurentAmps = leftExtentionMotor.getCurrentDrawAmps();

        inputs.spinPositionRad = leftExtentionMotor.getAngularPositionRad();
        inputs.spinAngularVelocity = leftExtentionMotor.getAngularVelocity();

        inputs.leftPositionRad = leftExtentionMotor.getAngularPositionRad();
        inputs.leftAngularVelocity = leftExtentionMotor.getAngularVelocity();
    }   

    @Override
    public void hangRunVoltage(double voltage) {
        hangAppliedVolts = voltage;
        leftExtentionMotor.setInputVoltage(voltage);
    }

    @Override
    public void spinRunVoltage(double voltage) {
        spinAppliedVolts = voltage;
        intakeSpinMotor.setInputVoltage(voltage);  

    }

}


inputs.articulationAppliedVolts = intakeRotateMotor.getAppliedOutput();
inputs.articulationCurrentAmps = intakeRotateMotor.getOutputCurrent();
