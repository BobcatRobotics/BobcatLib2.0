// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;

public class PrepareToSmoothie extends Command {
  private Intake intake;
  private boolean finished = false;
  private Shooter shooter;
  private Spivit spivit;

  /** Creates a new AutoIntake. */
  public PrepareToSmoothie(Intake intake, Shooter shooter, Spivit spivit) {
    this.intake = intake;
    this.shooter = shooter;
    this.spivit = spivit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, spivit);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    intake.removePeice();
    spivit.setAngle(ShooterConstants.stow);
    shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    if (!intake.hasPiece()) {
      intake.intakeToShooter();
    } else {
      intake.stop();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}