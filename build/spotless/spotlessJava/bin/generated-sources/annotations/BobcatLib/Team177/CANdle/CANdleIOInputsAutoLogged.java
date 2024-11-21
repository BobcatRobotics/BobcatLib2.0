package BobcatLib.Team177.CANdle;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CANdleIOInputsAutoLogged extends CANdleIO.CANdleIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("State", state);
  }

  @Override
  public void fromLog(LogTable table) {
    state = table.get("State", state);
  }

  public CANdleIOInputsAutoLogged clone() {
    CANdleIOInputsAutoLogged copy = new CANdleIOInputsAutoLogged();
    copy.state = this.state;
    return copy;
  }
}
