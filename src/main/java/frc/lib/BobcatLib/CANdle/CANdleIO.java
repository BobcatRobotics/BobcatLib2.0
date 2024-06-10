package frc.lib.BobcatLib.CANdle;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.BobcatLib.Annotations.SeasonSpecific;


@SeasonSpecific
public interface CANdleIO{
    @AutoLog
    public static class CANdleIOInputs {
        public CANdleState state = CANdleState.OFF;
    }

    public default void setLEDs(CANdleState state){}
    public default void updateInputs(CANdleIOInputs inputs){}
    

}
