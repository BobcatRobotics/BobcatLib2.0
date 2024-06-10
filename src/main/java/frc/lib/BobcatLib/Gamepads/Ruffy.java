package frc.lib.BobcatLib.Gamepads;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Ruffy{
    private final CommandJoystick gp;
    public Trigger button;
    public DoubleSupplier xAxis;
    public DoubleSupplier yAxis;
    public DoubleSupplier zAxis;

    /**
     * Logitech controller <p>
     * 
     * Buttons
     * 1 - x
     * 2 - a
     * 3-  b
     * 4 - y
     * 5 - lb
     * 6 - rb
     * 7 - lt
     * 8 - rt
     * 9 - back
     * 10 - start 
     * 11 - lstick
     * 12 - rstick <p>
     * 
     * Axes
     * 0 - lstick x
     * 1 - lstick y
     * 2 - rstick x
     * 3 - rstick y <p>
     * 
     * Axis indices start at 0, button indices start at one -_-
     */
    public Ruffy(int port) {
        gp = new CommandJoystick(port);
        configureTriggers();
        configureAxes();
    }

    private void configureTriggers(){
        button = gp.button(1);
    }

    private void configureAxes(){
        //y is up/down
        //x is left/right
        //z is twist
        xAxis = () -> -gp.getRawAxis(0);
        yAxis = () -> -gp.getRawAxis(1);
        zAxis = () -> -gp.getRawAxis(2);
    }



}
