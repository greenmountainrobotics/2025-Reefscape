import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO{
    @AutoLog

    class ClimberioInputs{
        public double inputs;
    }
    default void updateInputs(ClimberioInputs inputs){}

    default void setLeftVoltage(double voltage){}
}