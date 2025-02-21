package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj2.command.subsystemBase;
import org.littletonrobotics.junction.Logger;

public class CLimber extends subsystemBase {
    private final CLimberIo io;

    private final ClimberIoInputsAutoLogged inputs = new ClimberIoInputsAutoLogged();

    public Climber(ClimberIO io) {this.io=io;} 

    @Override 
    public void periodic(){
       io.updateInputs(inputs); 
       Logger.processInputs("Climber", inputs)
    }

    public void setVoltage(doubleVoltage){setVoltage(voltage, voltage);}

    public void setVoltage(double left){
        
    }
}