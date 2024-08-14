package BobcatLib.Team177.CANdle;

import com.ctre.phoenix.led.Animation;
import org.littletonrobotics.junction.AutoLog;

public interface CANdleIO {
  @AutoLog
  public static class CANdleIOInputs {
    public String state = "OFF";
  }

  public default void setLEDs(Animation animation, String name) {}

  public default void updateInputs(CANdleIOInputs inputs) {}
}
