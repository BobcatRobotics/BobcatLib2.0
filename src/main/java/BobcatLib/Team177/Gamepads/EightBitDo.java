package BobcatLib.Team177.Gamepads;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class EightBitDo {
  private final CommandJoystick gp;
  public Trigger a;
  public Trigger b;
  public Trigger x;
  public Trigger y;
  public Trigger lb;
  public Trigger rb;
  public Trigger select;
  public Trigger start;
  public Trigger leftPaddle;
  public Trigger rightPaddle;
  public DoubleSupplier leftXAxis;
  public DoubleSupplier leftYAxis;
  public DoubleSupplier rightXAxis;
  public DoubleSupplier rightYAxis;

  /**
   * 8bitdo controller
   *
   * <p>Buttons 1 -b 2 -a 3- y 4 -x 5 -lb 6 - rb 7 - select 8 - start 9 - bl 10 - br
   *
   * <p>Axes 0 - 1 - 2 - LT 3 - RT 4 - 5 -
   *
   * <p>Axis indices start at 0, button indices start at one -_-
   */
  public EightBitDo(int port) {
    gp = new CommandJoystick(port);
    configureTriggers();
    configureAxes();
  }

  private void configureTriggers() {
    a = gp.button(2);
    b = gp.button(1);
    x = gp.button(4);
    y = gp.button(3);
    lb = gp.button(5);
    rb = gp.button(6);
    select = gp.button(7);
    start = gp.button(8);
    leftPaddle = gp.button(9);
    rightPaddle = gp.button(10);
  }

  // TODO figure these out
  private void configureAxes() {
    leftXAxis = () -> gp.getRawAxis(0);
    leftYAxis = () -> gp.getRawAxis(0);
    rightXAxis = () -> gp.getRawAxis(0);
    rightYAxis = () -> gp.getRawAxis(0);
  }
}
