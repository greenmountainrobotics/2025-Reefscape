import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    public Climber(ClimberIO io) {this.io=io;} 

    public void setVoltage(double volts){
        setVoltage(volts);
    }
}