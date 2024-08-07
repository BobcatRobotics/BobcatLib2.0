package BobcatLib.Team177.CANdle;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;


public interface CANdleIO{
    @AutoLog
    public static class CANdleIOInputs {
        public String state = "OFF";
    }

    public default void setLEDs(Animation animation, String name){}
    public default void updateInputs(CANdleIOInputs inputs){}
    

}
