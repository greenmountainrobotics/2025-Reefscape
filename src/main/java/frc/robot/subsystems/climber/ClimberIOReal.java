package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.IdConstants.CANId;

public class ClimberIOReal{

    private final SparkFlex LeftHangMotor = new SparkFlex(CANId.LeftHangMotor, MotorType.kBrushless);
    private final SparkFlex RightHangMotor = new SparkFlex(CANId.RightHangMotor, MotorType.kBrushless);

    public void setVoltage(double volts){LeftHangMotor.setVoltage(volts); RightHangMotor.setVoltage(volts);}
}