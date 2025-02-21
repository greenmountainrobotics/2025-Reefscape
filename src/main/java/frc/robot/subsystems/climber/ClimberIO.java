package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO{
    @AutoLog

    class ClimberioInputs{
        public double 
    }
    default void updateInputs(ClimberioInputs inputs){}

    default void setLeftVoltage(double voltage){}
}