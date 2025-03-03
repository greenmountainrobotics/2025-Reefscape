package frc.robot.subsystems.intake;

import static frc.robot.constants.IdConstants.CANId.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.IdConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax intakeRotateMotor = new SparkMax(IntakeRotateMotorId, MotorType.kBrushed);
  private final SparkFlex intakeSpinMotor = new SparkFlex(IntakeSpinMotorId, MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(IdConstants.DIOId.LimitSwitchId);
  private final AbsoluteEncoder articulationEncoder;

  public IntakeIOReal() {
    articulationEncoder = intakeRotateMotor.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.articulationPosition =
        Rotation2d.fromRadians(
            Rotation2d.fromRotations(
                    (articulationEncoder
                        .getPosition() /*- IntakeConstants.AbsoluteEncoderOffsetRads*/))
                .getRadians());

    inputs.articulationPositionReal = Rotation2d.fromRadians(articulationEncoder.getPosition());

    inputs.articulationPositionRads = articulationEncoder.getPosition();

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
