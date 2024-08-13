package BobcatLib.Team177.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class CANdleIOCANdle implements CANdleIO {
    
    private CANdle leds;
    private String animName;
    public CANdleIOCANdle(int CANdleID){
        leds = new CANdle(CANdleID);
    }

    public void updateInputs(CANdleIOInputs inputs){
        inputs.state = animName;
    }

    /**
     * pass in a null animation to clear leds
     * leds should be cleared before passing in a new animation!
     */
    @Override
    public void setLEDs(Animation animation, String animationName){
       leds.animate(animation);
       animName = animationName;
    }


    
    
}