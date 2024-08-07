package BobcatLib.Team177;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import BobcatLib.Team177.CANdle.BuiltInAnimations;

public class BobcatUtil {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }

    public static boolean isBlue() {
        return getAlliance() == Alliance.Blue;
    }

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Animation getBuiltInAnimation(BuiltInAnimations animation, int ledCount) {
        switch (animation) {
            case ColorFlow:
                return new ColorFlowAnimation(128, 20, 70, 0, 0.7, ledCount, Direction.Forward);
            case Fire:
                return new FireAnimation(1, 0.7, ledCount, 0.7, 0.5);
            case Larson:
                return new LarsonAnimation(131, 23, 194, 0, 0.25, ledCount, BounceMode.Center, 2);
            case Rainbow:
                return new RainbowAnimation(1, 0.6, ledCount);
            case RgbFade:
                return new RgbFadeAnimation(0.7, 0.4, ledCount);
            case SingleFade:
                return new SingleFadeAnimation(50, 2, 200, 0, 0.5, ledCount);
            case Strobe:
                return new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, ledCount);
            case Twinkle:
                return new TwinkleAnimation(30, 70, 60, 0, 0.4, ledCount, TwinklePercent.Percent6);
            case TwinkleOff:
                return new TwinkleOffAnimation(70, 90, 175, 0, 0.8, ledCount,
                        TwinkleOffPercent.Percent100);
            default:
                return new StrobeAnimation(0, 0, 0);
        }}

    

    public static double get0to2Pi(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < (0)) {
            rad += (2 * Math.PI);
        } //should this be here?
        return rad;
    }
    public static double get0to2Pi(Rotation2d rot){
        return get0to2Pi(rot.getRadians());
    }
    /**
     * wraps the rotation2d to be within one rotation,
     * i.e. a rotation2d with a value of 370 degrees will return a 
     * rotation2d with a value of 10 degrees
     */
    public static Rotation2d wrapRot2d(Rotation2d rot){
        return Rotation2d.fromRadians(get0to2Pi(rot));
    }


}
