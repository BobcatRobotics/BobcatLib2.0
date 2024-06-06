package frc.lib.BobcatLib.CANdle;

import com.ctre.phoenix.led.CANdle;

import frc.lib.BobcatLib.Annotations.SeasonSpecific;
import frc.robot.Constants.CANdleConstants;

@SeasonSpecific
public class CANdleIOCANdle implements CANdleIO {
    
    private CANdle leds;
    private CANdleState currState = CANdleState.OFF;
    public CANdleIOCANdle(){
        leds = new CANdle(CANdleConstants.CANdleID);
    }

    public void updateInputs(CANdleIOInputs inputs){
        inputs.state = currState;
    }

    @Override
    public void setLEDs(CANdleState state){
        if(state != currState){
            leds.animate(null); //wipe old state when setting new one
        }
        currState = state;
        

        leds.animate(state.getAnimation());
    }


    
    
}