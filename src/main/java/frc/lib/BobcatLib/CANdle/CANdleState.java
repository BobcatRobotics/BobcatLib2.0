package frc.lib.BobcatLib.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.lib.BobcatLib.Annotations.SeasonSpecific;
import frc.robot.Constants.CANdleConstants;
import lombok.AllArgsConstructor;
import lombok.Getter;



/**
 * add season-specific states here
 */
@SeasonSpecific
@AllArgsConstructor
public enum CANdleState {
    OFF(null),
    INTAKESTALL(new StrobeAnimation(255, 0, 0,  0, 0.25, CANdleConstants.LedCount)),
    RESETPOSE(new StrobeAnimation(255, 170, 0,  0, 0.25, CANdleConstants.LedCount)),
    RESETGYRO(new StrobeAnimation(255, 170, 0, 0, 0.25, CANdleConstants.LedCount)),
    OUTAKE(new StrobeAnimation(255, 0, 0, 0, 0.75, CANdleConstants.LedCount)),
    ALIGNING(new StrobeAnimation(255, 255, 255, 255, 0.75, CANdleConstants.LedCount)),
    ALIGNED(new StrobeAnimation(0, 0, 255, 0, 1, CANdleConstants.LedCount));
    
    @Getter private Animation animation; 
}



