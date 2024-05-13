// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.VisionConstants;
import frc.lib.BobcatLib.Gamepads.EightBitDo;
import frc.lib.BobcatLib.Swerve.GyroIO;
import frc.lib.BobcatLib.Swerve.GyroIOPigeon2;
import frc.lib.BobcatLib.Swerve.SwerveBase;
import frc.lib.BobcatLib.Swerve.TeleopSwerve;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIO;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIOFalcon;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIOSim;
import frc.lib.BobcatLib.Sysid.Sysid;
import frc.lib.BobcatLib.Sysid.SysidTest;
import frc.lib.BobcatLib.Vision.Vision;
import frc.lib.BobcatLib.Vision.VisionIO;
import frc.lib.BobcatLib.Vision.VisionIOLimelight;
import frc.robot.Constants.SwerveConstants;

public class RobotContainer {

        /* Joysticks + Gamepad */
        private final EightBitDo gp = new EightBitDo(0);
        private final CommandJoystick rotate = new CommandJoystick(1);
        private final CommandJoystick translate = new CommandJoystick(0);


        /* Subsystems */
        public final SwerveBase swerve;
        public final Vision limelight1;
        public final Vision[] cameras;
        
        public final Sysid sysid;
        /* Commands */

        /* Shuffleboard Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        case REAL:
                                limelight1 = new Vision(new VisionIOLimelight(VisionConstants.limelight1.constants));
                                cameras = new Vision[]{limelight1};
                                
                                swerve = new SwerveBase(new GyroIOPigeon2(),
                                                new SwerveModuleIOFalcon(SwerveConstants.Module0Constants.constants), //fl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module1Constants.constants), //fr
                                                new SwerveModuleIOFalcon(SwerveConstants.Module2Constants.constants), //bl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module3Constants.constants), //br
                                                cameras);
                                sysid = new Sysid(swerve);
                                break;

                        // Sim robot, instantiate physics sim IO implementations
                        case SIM:
                                limelight1 = new Vision(new VisionIOLimelight(VisionConstants.limelight1.constants));
                                cameras = new Vision[]{limelight1};

                                swerve = new SwerveBase(new GyroIO() {
                                },
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                cameras);
                                sysid = new Sysid(swerve);
                                break;

                        // Replayed robot, disable IO implementations
                        default:
                                //TODO limelight replay capability
                                limelight1 = new Vision(new VisionIO() {
                                });
                                cameras = new Vision[]{limelight1};
                                swerve = new SwerveBase(new GyroIO() {
                                },
                                                new SwerveModuleIOSim() {
                                                },
                                                new SwerveModuleIOSim() {
                                                },
                                                new SwerveModuleIOSim() {
                                                },
                                                new SwerveModuleIOSim() {
                                                },
                                                cameras);
                                sysid = new Sysid(swerve);
                                break;

                }
                configureBindings();
        }

        public boolean autoChooserInitialized(){
                return autoChooser.get() != null;
        }

        /**
         * this should only be called once DS and FMS are attached
         */
        public void configureAutos(){
                
                /*
                 * Auto Events
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                NamedCommands.registerCommand("PathfindingCommand", swerve.driveToPose(new Pose2d()));

                /*
                 * Auto Chooser
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                autoChooser.addDefaultOption("Do Nothing", Commands.none());
                autoChooser.addOption("ur mom", new ParallelDeadlineGroup(new WaitCommand(10), new RunCommand(() -> swerve.drive(new Translation2d(1,1), 0, false, false), swerve)));
        }

        /**
         * IMPORTANT NOTE:
         * When a gamepad value is needed by a command, don't
         * pass the gamepad to the command, instead have the
         * constructor for the command take an argument that
         * is a supplier of the value that is needed. To supply
         * the values, use an anonymous/lambda function like this:
         * 
         * () -> buttonOrAxisValue
         */
        public void configureBindings() {

                swerve.setDefaultCommand(
                   new TeleopSwerve(
                        swerve,
                        () -> -translate.getRawAxis(Joystick.AxisType.kY.value),
                        () -> -translate.getRawAxis(Joystick.AxisType.kX.value), 
                        () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), 
                        () -> false,
                        () -> -rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.25, 
                        () -> -translate.getRawAxis(Joystick.AxisType.kZ.value) * 0.25, 
                        gp.lb, 
                        gp.rb
                        )
                );
                
                //sysid routines
                gp.a.whileTrue(sysid.getSysidTest(SysidTest.QUASISTATIC_FORWARD));
                gp.b.whileTrue(sysid.getSysidTest(SysidTest.QUASISTATIC_BACKWARD));
                gp.x.whileTrue(sysid.getSysidTest(SysidTest.DYNAMIC_FORWARD));
                gp.y.whileTrue(sysid.getSysidTest(SysidTest.DYNAMIC_BACKWARD));
                //TODO button to set all wheels straight
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
