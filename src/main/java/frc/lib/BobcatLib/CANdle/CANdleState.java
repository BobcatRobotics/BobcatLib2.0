package frc.lib.BobcatLib.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.lib.BobcatLib.Annotations.SeasonSpecific;
import frc.robot.Constants.CANdleConstants;
import lombok.AllArgsConstructor;
import lombok.Getter;



/**
 * add season-specific states here
 * built in animations are 
 * ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff
 */
@SeasonSpecific
@AllArgsConstructor
public enum CANdleState {
    OFF(null),
    INTAKESTALL(new StrobeAnimation(255, 255, 255,  255, 0.25, CANdleConstants.LedCount)),
    RESETPOSE(new StrobeAnimation(255, 255, 255,  255, 0.25, CANdleConstants.LedCount)),
    RESETGYRO(new StrobeAnimation(255, 255, 255,  255, 0.25, CANdleConstants.LedCount)),
    OUTAKE(new StrobeAnimation(255, 255, 255,  255, 0.25, CANdleConstants.LedCount)),
    ALIGNING(new StrobeAnimation(255, 255, 255,  255, 0.25, CANdleConstants.LedCount)),
    ALIGNED(new StrobeAnimation(255, 255, 255,  255, 0.25, CANdleConstants.LedCount));
    
    @Getter private Animation animation; 
}



