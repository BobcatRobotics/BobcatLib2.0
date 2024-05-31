// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.BobcatLib.CANdle;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
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
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BobcatLib.Annotations.SeasonSpecific;
import frc.robot.Constants.CANdleConstants;

@SeasonSpecific
public class CANdle extends SubsystemBase {
  private final CANdleIO io;
  private final CANdleIOInputsAutoLogged inputs = new CANdleIOInputsAutoLogged();
  private double seconds = 1;
  private Timer timer = new Timer();

  /** Creates a new CANdle. */
  public CANdle(CANdleIO candle) {
    io = candle;
    timer.reset();

  }

  /**
   * THIS WILL NOT AUTOMATICALLY TURN OFF, it will persist untill you set it again
   * 
   * @param state the animation to play
   */
  public void setLEDs(CANdleState state) {
    timer.stop();
    timer.reset();
    io.setLEDs(state);
  }

  /**
   * 
   * @param state   the animation to play
   * @param seconds duration to play
   */
  public void setLEDs(CANdleState state, double seconds) {
    setLEDs(state);
    this.seconds = seconds;
    timer.reset();
    timer.start();
  }

  public CANdleState getState() {
    return inputs.state;
  }

  @Override
  public void periodic() {

    if (timer.hasElapsed(seconds)) {
      setLEDs(CANdleState.OFF);
      timer.stop();
      timer.reset();
      seconds = 1;
    }

    io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
  }

  public static Animation getBuiltInAnimation(BuiltInAnimations animation) {
    switch (animation) {
      case ColorFlow:
        return new ColorFlowAnimation(128, 20, 70, 0, 0.7, CANdleConstants.LedCount, Direction.Forward);
      case Fire:
        return new FireAnimation(1, 0.7, CANdleConstants.LedCount, 0.7, 0.5);
      case Larson:
        return new LarsonAnimation(131, 23, 194, 0, 0.25, CANdleConstants.LedCount, BounceMode.Center, 2);
      case Rainbow:
        return new RainbowAnimation(1, 0.6, CANdleConstants.LedCount);
      case RgbFade:
        return new RgbFadeAnimation(0.7, 0.4, CANdleConstants.LedCount);
      case SingleFade:
        return new SingleFadeAnimation(50, 2, 200, 0, 0.5, CANdleConstants.LedCount);
      case Strobe:
        return new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, CANdleConstants.LedCount);
      case Twinkle:
        return new TwinkleAnimation(30, 70, 60, 0, 0.4, CANdleConstants.LedCount, TwinklePercent.Percent6);
      case TwinkleOff:
        return new TwinkleOffAnimation(70, 90, 175, 0, 0.8, CANdleConstants.LedCount,
            TwinkleOffPercent.Percent100);
      default:
        return new StrobeAnimation(0, 0, 0);
    }
  }

}
