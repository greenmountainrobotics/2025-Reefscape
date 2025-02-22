package frc.robot.subsystems.intake;

import static frc.robot.constants.IdConstants.CANId.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Import for motor types


public class IntakeIOReal implements IntakeIO {
  private final CANSparkMax intakeRotateMotor = new CANSparkMax(0, motorType);
}
