<<<<<<< Updated upstream
package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.IdConstants.CANId;

public class ClimberIOReal implements ClimberIO {

  private final SparkFlex leftHangMotor = new SparkFlex(CANId.LeftHangMotor, MotorType.kBrushless);
  private final SparkFlex rightHangMotor =
      new SparkFlex(CANId.RightHangMotor, MotorType.kBrushless);

  public ClimberIOReal() {}

  public void setVoltage(double volts) {
    leftHangMotor.setVoltage(volts);
    rightHangMotor.setVoltage(-volts);
  }
}
=======
// ihttps://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Motor%20Follower/src/main/java/frc/robot/Robot.java\
// replaceing teh vitor sp
>>>>>>> Stashed changes
