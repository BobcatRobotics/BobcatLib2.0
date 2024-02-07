// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.AlignToTag;
import frc.robot.Commands.DriveToPose;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.GrabNote;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveModuleIO;
import frc.robot.Subsystems.Swerve.SwerveModuleIOFalcon;
import frc.robot.Subsystems.Swerve.SwerveModuleIOSim;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOLimelight;

public class RobotContainer {
  /* Joysticks + Gamepad */
  private final CommandJoystick rotate = new CommandJoystick(1);
  private final CommandJoystick strafe = new CommandJoystick(0);
  private final CommandJoystick gp = new CommandJoystick(2);
  

  /* Subsystems */
  public final Swerve m_swerve;
  public final Vision m_Vision;

  /* Commands */

  /* Shuffleboard Inputs */
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        m_swerve = new Swerve(new GyroIOPigeon2(),
            new SwerveModuleIOFalcon(SwerveConstants.Module0Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module1Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module2Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module3Constants.constants));
        m_Vision = new Vision(new VisionIOLimelight());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        m_swerve = new Swerve(new GyroIO() {
        },
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());
            m_Vision = new Vision(new VisionIOLimelight());
        break;

      // Replayed robot, disable IO implementations
      default:
        m_swerve = new Swerve(new GyroIO() {
        },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            });
            m_Vision = new Vision(new VisionIO(){});
        break;
    }

    /* Auto Chooser
     * 
     * Names must match what is in PathPlanner
     * Please give descriptive names
    */
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("outta the way", new PathPlannerAuto("Auto 1"));
    autoChooser.addOption("straight line", new PathPlannerAuto("straight line"));

    /* Auto Events
     * 
     * Names must match what is in PathPlanner
     * Please give descriptive names
    */
    NamedCommands.registerCommand("Auto Event", new InstantCommand());

    configureBindings();

  }

  /*
   * IMPORTANT NOTE:
   * When a gamepad value is needed by a command, don't
   * pass the gamepad to the command, instead have the
   * constructor for the command take an argument that
   * is a supplier of the value that is needed. To supply
   * the values, use an anonymous function like this:
   * 
   * () -> buttonOrAxisValue
   */
  private void configureBindings() {
    /* Drive with joysticks */
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> strafe.getRawAxis(Joystick.AxisType.kY.value)
                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)),
            () -> strafe.getRawAxis(Joystick.AxisType.kX.value)
                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)),
            () -> rotate.getRawAxis(Joystick.AxisType.kX.value),
            () -> false,
            () -> rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine tune
            () -> strafe.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine tune
            // strafe.button(1) 
            () -> false
        ));
      rotate.button(1).onTrue(new InstantCommand(m_swerve::zeroGyro));

      // rotate.button(1).onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));

      strafe.button(1).whileTrue(new AlignToTag(m_swerve, m_Vision, null));
      

    /* Drive with gamepad */
    // m_swerve.setDefaultCommand(
    //     new TeleopSwerve(
    //         m_swerve,
    //         () -> -gp.getRawAxis(Joystick.AxisType.kY.value)
    //             * Math.abs(gp.getRawAxis(Joystick.AxisType.kY.value)),
    //         () -> -gp.getRawAxis(Joystick.AxisType.kX.value)
    //             * Math.abs(gp.getRawAxis(Joystick.AxisType.kX.value)),
    //         () -> -gp.getRawAxis(Joystick.AxisType.kZ.value),
    //         () -> false,
    //         () -> 0.0,
    //         () -> 0.0
    //     ));

        
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
