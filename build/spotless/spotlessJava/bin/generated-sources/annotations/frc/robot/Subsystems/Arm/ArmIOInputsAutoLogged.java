package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Angle", angle);
  }

  @Override
  public void fromLog(LogTable table) {
    angle = table.get("Angle", angle);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.angle = this.angle;
    return copy;
  }
}
