// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.BobcatLib.Gamepads.EightBitDo;
import frc.lib.util.BobcatLib.Swerve.GyroIO;
import frc.lib.util.BobcatLib.Swerve.GyroIOPigeon2;
import frc.lib.util.BobcatLib.Swerve.Swerve;
import frc.lib.util.BobcatLib.Swerve.SwerveModule.SwerveModuleIO;
import frc.lib.util.BobcatLib.Swerve.SwerveModule.SwerveModuleIOFalcon;
import frc.lib.util.BobcatLib.Swerve.SwerveModule.SwerveModuleIOSim;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOLimelight;

public class RobotContainer {

        /* Joysticks + Gamepad */
        private final EightBitDo gp = new EightBitDo(0);

        /* Subsystems */
        public final Swerve swerve;
        public final Vision limelight1;
        public final Vision[] cameras;
        /* Commands */

        /* Shuffleboard Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        case REAL:
                                limelight1 = new Vision(new VisionIOLimelight(LimelightConstants.limelight1.constants));
                                cameras = new Vision[]{limelight1};
                                swerve = new Swerve(new GyroIOPigeon2(),
                                                new SwerveModuleIOFalcon(SwerveConstants.Module0Constants.constants), //fl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module1Constants.constants), //fr
                                                new SwerveModuleIOFalcon(SwerveConstants.Module2Constants.constants), //bl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module3Constants.constants), //br
                                                cameras);
                                break;

                        // Sim robot, instantiate physics sim IO implementations
                        case SIM:
                                limelight1 = new Vision(new VisionIOLimelight(LimelightConstants.limelight1.constants));
                                cameras = new Vision[]{limelight1};

                                swerve = new Swerve(new GyroIO() {
                                },
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                cameras);

                                break;

                        // Replayed robot, disable IO implementations
                        default:
                                //TODO limelight replay capability
                                limelight1 = new Vision(new VisionIO() {
                                });
                                cameras = new Vision[]{limelight1};
                                swerve = new Swerve(new GyroIO() {
                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                cameras);
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
                
                //will run when 'a' button is pressed
                gp.a.onTrue(
                        new InstantCommand(
                                () -> {System.out.println("example");}
                        )
                );
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
