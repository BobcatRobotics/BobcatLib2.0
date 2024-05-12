// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;



import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.BobcatLib.Swerve.Swerve;
import frc.robot.Subsystems.CANdle.CANdle;
import frc.robot.Subsystems.CANdle.CANdleState;
import frc.robot.Subsystems.Spivit.Spivit;

public class CandleAlignment extends Command {
  Spivit spivit;
  Swerve swerve;
  CANdle candle;
  BooleanSupplier feeding;

  /** Creates a new CandleAlignment. */
  public CandleAlignment(Spivit spivit, Swerve swerve, CANdle candle, BooleanSupplier feeding) {
    this.spivit = spivit;
    this.swerve = swerve;
    this.candle = candle;
    this.feeding = feeding;

    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(feeding.getAsBoolean()){
      candle.setLEDs(CANdleState.FEED);
    }else if (spivit.aligned() && swerve.alignedLEDS(spivit.getDesiredAngle())) {
      candle.setLEDs(CANdleState.ALIGNED);
    } else {
      candle.setLEDs(CANdleState.ALIGNING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    candle.setLEDs(CANdleState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
