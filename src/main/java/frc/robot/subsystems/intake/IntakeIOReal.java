package frc.robot.subsystems.intake;

import static frc.robot.constants.IdConstants.CANId.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.IdConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax intakeRotateMotor = new SparkMax(IntakeRotateMotorId, MotorType.kBrushed);
  SparkMaxConfig config = new SparkMaxConfig();
  private final SparkFlex intakeSpinMotor = new SparkFlex(IntakeSpinMotorId, MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(IdConstants.DIOId.LimitSwitchId);
  private final AbsoluteEncoder articulationEncoder;

  public IntakeIOReal() {
    articulationEncoder = intakeRotateMotor.getAbsoluteEncoder();
    config.inverted(true).idleMode(IdleMode.kBrake);

    intakeRotateMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.articulationPosition = new Rotation2d(articulationEncoder.getPosition());

    inputs.articulationPositionReal = new Rotation2d(articulationEncoder.getPosition());

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
