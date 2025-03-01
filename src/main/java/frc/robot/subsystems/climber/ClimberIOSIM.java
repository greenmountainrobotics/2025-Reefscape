//copied from real

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

// Copied from the last years code

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

