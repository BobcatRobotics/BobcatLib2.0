// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.BobcatLib.Gamepads.Logitech;
import frc.lib.BobcatLib.Swerve.GyroIO;
import frc.lib.BobcatLib.Swerve.GyroIOPigeon2;
import frc.lib.BobcatLib.Swerve.SwerveConstants;
import frc.lib.BobcatLib.Swerve.TeleopSwerve;
import frc.lib.BobcatLib.Swerve.Assists.RotationalAssist;
import frc.lib.BobcatLib.Swerve.Assists.TranslationAssist;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIOFalcon;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIOSim;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {

        /* Joysticks + Gamepad */
        private final Logitech gp = new Logitech(0);
        


        /* Subsystems */
        public final Swerve swerve;
        //public Vision limelight1;
        //public Vision[] cameras;
        
        /* Commands */

        /* Shuffleboard Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        case REAL:
                                //limelight1 = new Vision(new VisionIOLimelight("limelight1", LimeLightType.LL3G_APRILTAG));
                               // cameras = new Vision[]{limelight1};
                                
                                swerve = new Swerve(new GyroIOPigeon2(),
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module0Constants.constants), //fl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module1Constants.constants), //fr
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module2Constants.constants), //bl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module3Constants.constants) //br
                                                );
                                break;

                        // Sim robot, instantiate physics sim IO implementations
                        case SIM:
                                //limelight1 = new Vision(new VisionIOLimelight("limelight1", LimeLightType.LL3G_APRILTAG));
                                //cameras = new Vision[]{limelight1};

                                swerve = new Swerve(new GyroIO() {
                                },
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs),
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs),
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs),
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs)
                                                );


                                break;

                        // Replayed robot, disable IO implementations
                        default:
                                //limelight1 = new Vision(new VisionIO() {
                                //});
                                //cameras = new Vision[]{limelight1};
                                swerve = new Swerve(new GyroIO() {
                                },
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs) {
                                                },
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs) {
                                                },
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs) {
                                                },
                                                new SwerveModuleIOSim(Constants.loopPeriodSecs) {
                                                });
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
                //NamedCommands.registerCommand("PathfindingCommand", swerve.driveToPose(new Pose2d()));

                /*
                 * Auto Chooser
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                autoChooser.addDefaultOption("Do Nothing", Commands.none());
                autoChooser.addOption("test", new PathPlannerAuto("New Auto"));
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
                swerve.setAimAssistTranslation(new Translation2d(5,5));
                TranslationAssist transAssist = new TranslationAssist(() -> swerve.getAimAssistTranslation(),() -> swerve.getPose().getTranslation(), gp.a, () -> false);
                RotationalAssist rotAssist = new RotationalAssist(() -> new Rotation2d(), () -> swerve.getYaw(), () -> false, () -> false);
                swerve.setDefaultCommand(
                   new TeleopSwerve(
                        swerve,
                        gp.leftYAxis,
                        gp.leftXAxis, 
                        gp.rightXAxis, 
                        gp.rb,
                        () -> 0 , 
                        () -> 0, 
                        transAssist, 
                        rotAssist
                        ));
                
                
                //sysid routines
                gp.start.onTrue(new InstantCommand(() -> swerve.resetPose(new Pose2d())));
                gp.back.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
               
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }


        // public Pose3d getArmPoseAScope(){
        //         Rotation2d angle = Rotation2d.fromRotations(rotate.getRawAxis(2));
        //         return new Pose3d(0, -0.16, 0.23, new Rotation3d(angle.getRadians(), 0, 0));
        // }
}
