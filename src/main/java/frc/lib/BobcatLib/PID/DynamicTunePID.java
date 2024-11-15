package frc.lib.BobcatLib.PID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.BobcatLib.Annotations.SeasonBase;

@SeasonBase
public class DynamicTunePID extends PIDController {
    private double p;
    private double i;
    private double d;

    /**
     * @deprecated
     * TODO: WIP <p>
     * 
     * NOT FOR USE IN COMP <n>
     * Should automatically publish pid to nt for tuning purposes,<n> 
     * robot will prob need to be in test mode for this to work
     */
    @Deprecated
    public DynamicTunePID(double kp, double ki, double kd){
        super(kp, ki, kd);
        p = kp;
        i = ki;
        d = kd;
        Shuffleboard.getTab("tuning").add(this).withWidget(BuiltInWidgets.kPIDController);
    }



    
    
}
