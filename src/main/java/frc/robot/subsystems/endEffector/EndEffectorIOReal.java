package frc.robot.subsystems.endEffector;

import static frc.robot.constants.IdConstants.CANId.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;

public class EndEffectorIOReal implements EndEffectorIO {
  private final SparkMax intakeRotateMotor =
      new SparkMax(EndEffectorRotateMotorId, MotorType.kBrushless);
  private final SparkMax intakeSpinMotor = new SparkMax(EndEffectorSpinMotorId, MotorType.kBrushed);

  private final DigitalInput limitSwitch = new DigitalInput(IdConstants.DIOId.LimitSwitchId);
  private final AbsoluteEncoder articulationEncoder;

  public EndEffectorIOReal() {
    articulationEncoder = intakeRotateMotor.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.articulationPosition =
        Rotation2d.fromRadians(
            Rotation2d.fromRotations(
                    (-articulationEncoder.getPosition()
                        + IntakeConstants.AbsoluteEncoderOffset.getRotations()))
                .getRadians());

    inputs.articulationAppliedVolts = intakeRotateMotor.getAppliedOutput();
    inputs.articulationCurrentAmps = intakeRotateMotor.getOutputCurrent();

    inputs.spinAppliedVolts = intakeSpinMotor.getAppliedOutput();
    inputs.spinCurrentAmps = intakeSpinMotor.getOutputCurrent();
    inputs.limitSwitchPressed = limitSwitch.get();
  }

  @Override
  public void articulationRunVoltage(double voltage) {
    intakeRotateMotor.setVoltage(voltage);
  }

  @Override
  public void spinRunVoltage(double voltage) {
    intakeSpinMotor.setVoltage(voltage);
  }
}
