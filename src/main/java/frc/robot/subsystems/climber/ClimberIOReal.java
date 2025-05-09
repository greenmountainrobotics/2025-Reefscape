package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IdConstants.CANId;

public class ClimberIOReal implements ClimberIO {

  private final SparkFlex leftHangMotor = new SparkFlex(CANId.LeftHangMotor, MotorType.kBrushless);
  // private final SparkFlex rightHangMotor = new SparkFlex(CANId.RightHangMotor,
  // MotorType.kBrushless);

  public ClimberIOReal() {}

  @Override
  public void setClimbVoltage(double volts) {
    leftHangMotor.setVoltage(volts);
    //  rightHangMotor.setVoltage(-volts);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftCurrentAmps = leftHangMotor.getOutputCurrent();
    //   inputs.rightCurrentAmps = rightHangMotor.getOutputCurrent();

    inputs.leftAppliedVolts = leftHangMotor.getAppliedOutput();
    //   inputs.rightAppliedVolts = rightHangMotor.getAppliedOutput();

    //  inputs.encoderRight = rightHangMotor.getAppliedOutput();
    inputs.encoderLeft = leftHangMotor.getAppliedOutput();
  }
}
