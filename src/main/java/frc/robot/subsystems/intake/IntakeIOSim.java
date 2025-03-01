package frc.robot.subsystems.intake;


//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class IntakeIOSim implements IntakeIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotor intakeSpinMotor = DCMotor.getVex775Pro(1);
    private final DCMotor intakeRotateMotor = DCMotor.getNEO(1);

    private DCMotorSim intakeSpinMotorSim = 
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            intakeSpinMotor, 1.0, 100),
            intakeSpinMotor);
    private DCMotorSim intakeRotateMotorSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            intakeRotateMotor, 1.0, 80),
            intakeRotateMotor);

    private double spinAppliedVolts = 0.0;
    private double spinCurrentAmps = 0.0;
    
    private double articulationAppliedVolts = 0.0;
    private double articulationCurrentAmps = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeSpinMotorSim.update(LOOP_PERIOD_SECS);
        intakeRotateMotorSim.update(LOOP_PERIOD_SECS);

        //inputs.articulationPosition  = intakeSpinMotor.();
        /*
        private double intakeSpinMotorVelocityRadPerSec = intakeSpinMotor.getAngularVelocityRadPerSec();
        private double intakeRotateMotorPositionRad = intakeRotateMotor.getAngularPositionRad();
        private double intakeRotateMotorVelocityRadPerSec = intakeRotateMotor.getAngularVelocityRadPerSec();
        */
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
}

    @Override
    public void articulationRunVoltage(double voltage) {
        articulationAppliedVolts = voltage;
        intakeRotateMotorSim.setInputVoltage(voltage); //why setINPUTVoltage?
    }

    @Override
    public void spinRunVoltage(double voltage) {
        spinAppliedVolts = voltage;
        intakeSpinMotorSim.setInputVoltage(voltage);  //why setINPUTVoltage?
    }
}