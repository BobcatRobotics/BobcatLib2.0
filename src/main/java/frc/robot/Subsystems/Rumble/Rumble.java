// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Rumble;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Rumble extends SubsystemBase {

  // im absolutely not putting the effort to add advantagekit to log when the joystick is rumbling

  XboxController rumbleController;
  Timer timer;

  /** Creates a new Rumble. */
  public Rumble() {
    rumbleController = new XboxController(2);
  }

  /**
   * 
   * @param  [-1,1]
   * @param time seconds
   * @return command that rumbles the joystick
   */
  public Command rumble(double intensity, double time){
    return new 
      ParallelRaceGroup(new InstantCommand(() -> rumbleController.setRumble(RumbleType.kBothRumble, intensity)), new WaitCommand(time));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
