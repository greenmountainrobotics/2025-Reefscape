package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.constants.IdConstants;

public class ClimberIoReal implements ClimberIO{

    private final sparkFlex LeftHangMotor = new sparkFlex(IdConstants.LeftHangMotor);

    @Override
    public void updateInputs(ClimberioInputs){
        inputs.LeftHangMotor = LeftHangMotor.get();
    }

    @Override
    public void setLeftHangVoltage(double volts){LeftHangMotor.setVoltage(volts;)}
}