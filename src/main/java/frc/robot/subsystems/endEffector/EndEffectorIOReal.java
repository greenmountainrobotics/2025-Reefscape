package frc.robot.subsystems.endEffector;

import static frc.robot.constants.IdConstants.CANId.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.IdConstants;

public class EndEffectorIOReal implements EndEffectorIO {
  private final SparkMax intakeRotateMotor =
      new SparkMax(EndEffectorRotateMotorId, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  private final SparkMax intakeSpinMotor = new SparkMax(EndEffectorSpinMotorId, MotorType.kBrushed);
  private double Voltage = 0.0;

  private final DigitalInput limitSwitch =
      new DigitalInput(IdConstants.DIOId.EndEffectorLimitSwitchId);
  private final AbsoluteEncoder articulationEncoder;

  public EndEffectorIOReal() {
    articulationEncoder = intakeRotateMotor.getAbsoluteEncoder();

    config.inverted(true).idleMode(IdleMode.kBrake);

    intakeRotateMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.articulationPosition = new Rotation2d(articulationEncoder.getPosition() * 2 * Math.PI);

    inputs.articulationAppliedVolts = intakeRotateMotor.getAppliedOutput();
    inputs.articulationCurrentAmps = intakeRotateMotor.getOutputCurrent();

    inputs.spinAppliedVolts = Voltage;
    inputs.spinCurrentAmps = intakeSpinMotor.getOutputCurrent();
    inputs.limitSwitchPressed = limitSwitch.get();
  }

  @Override
  public void articulationRunVoltage(double voltage) {
    intakeRotateMotor.setVoltage(voltage * 12);
  }

  @Override
  public void spinRunVoltage(double voltage) {
    intakeSpinMotor.setVoltage(voltage);
  }
}
