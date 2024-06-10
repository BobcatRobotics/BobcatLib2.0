package frc.lib.BobcatLib.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.lib.BobcatLib.Annotations.SeasonSpecific;
import lombok.AllArgsConstructor;
import lombok.Getter;



/**
 * add season-specific states here
 */
@SeasonSpecific
@AllArgsConstructor
public enum CANdleState {
    OFF(null),
    INTAKESTALL(new StrobeAnimation(255, 0, 0,  0, 0.25, CANdleConstants.LEDCount)),
    RESETPOSE(new StrobeAnimation(255, 170, 0,  0, 0.25, CANdleConstants.LEDCount)),
    RESETGYRO(new StrobeAnimation(255, 170, 0, 0, 0.25, CANdleConstants.LEDCount)),
    OUTAKE(new StrobeAnimation(255, 0, 0, 0, 0.75, CANdleConstants.LEDCount)),
    ALIGNING(new StrobeAnimation(255, 255, 255, 255, 0.75, CANdleConstants.LEDCount)),
    ALIGNED(new StrobeAnimation(0, 0, 255, 0, 1, CANdleConstants.LEDCount));
    
    @Getter private Animation animation; 
}



