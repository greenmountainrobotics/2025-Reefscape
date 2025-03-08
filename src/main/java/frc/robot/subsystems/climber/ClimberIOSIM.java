package frc.robot.subsystems.climber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSIM implements ClimberIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final DCMotor leftHangMotor = DCMotor.getVex775Pro(1);
    private static final DCMotor rightHangMotor = DCMotor.getNEO(1);

    private DCMotorSim leftHangMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leftHangMotor, 1.0, 1.0), leftHangMotor);

    private DCMotorSim rightHangMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rightHangMotor, 1.0, 1.0), rightHangMotor);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double leftCurrentAmps = 0.0;
    private double rightCurrentAmps = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        leftHangMotorSim.update(LOOP_PERIOD_SECS);
        rightHangMotorSim.update(LOOP_PERIOD_SECS);

        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = leftCurrentAmps;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = rightCurrentAmps;

        inputs.leftAppliedVolts = leftHangMotorSim.getInputVoltage();
        inputs.leftCurrentAmps = leftHangMotorSim.getCurrentDrawAmps();

        inputs.leftPositionRad = leftHangMotorSim.getAngularPositionRad();
        inputs.leftAngularVelocity = leftHangMotorSim.getAngularVelocity();
    }   

    @Override
    public void leftHangRunVoltage(double voltage) {
        leftAppliedVolts = voltage;
        leftHangMotorSim.setInputVoltage(voltage);
    }

    @Override
    public void rightHangRunVoltage(double voltage) {
        rightAppliedVolts = voltage;
        rightHangMotorSim.setInputVoltage(voltage);  

    }
}