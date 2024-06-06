// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.lib.BobcatLib.Gamepads.EightBitDo;
import frc.lib.BobcatLib.Swerve.GyroIO;
import frc.lib.BobcatLib.Swerve.GyroIOPigeon2;
import frc.lib.BobcatLib.Swerve.SwerveConstants;
import frc.lib.BobcatLib.Swerve.TeleopSwerve;
import frc.lib.BobcatLib.Swerve.Assists.RotationalAssist;
import frc.lib.BobcatLib.Swerve.Assists.TranslationalAssist;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIOFalcon;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIOSim;
import frc.lib.BobcatLib.Sysid.Sysid;
import frc.lib.BobcatLib.Vision.Vision;
import frc.lib.BobcatLib.Vision.VisionIO;
import frc.lib.BobcatLib.Vision.VisionIOLimelight;
import frc.lib.BobcatLib.Vision.VisionConstants.LimeLightType;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {

        /* Joysticks + Gamepad */
        private final EightBitDo gp = new EightBitDo(0);
        private final CommandJoystick rotate = new CommandJoystick(1);
        private final CommandJoystick translate = new CommandJoystick(0);


        /* Subsystems */
        public final Swerve swerve;
        public final Vision limelight1;
        public final Vision[] cameras;
        
        public final Sysid sysid;
        
        /* Driver Assists */
        TranslationalAssist transAssist;
        RotationalAssist rotAssist;
        

        /* NetworkTable Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        case REAL:
                                limelight1 = new Vision(new VisionIOLimelight("limelight1", LimeLightType.LL3G_APRILTAG));
                                cameras = new Vision[]{limelight1};
                                
                                swerve = new Swerve(new GyroIOPigeon2(),
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module0Constants.constants), //fl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module1Constants.constants), //fr
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module2Constants.constants), //bl
                                                new SwerveModuleIOFalcon(SwerveConstants.Module.Module3Constants.constants), //br
                                                cameras);
                                sysid = new Sysid(swerve);
                                break;

                        // Sim robot, instantiate physics sim IO implementations
                        case SIM:
                                limelight1 = new Vision(new VisionIOLimelight("limelight1", LimeLightType.LL3G_APRILTAG));
                                cameras = new Vision[]{limelight1};

                                swerve = new Swerve(new GyroIO() {
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
                                limelight1 = new Vision(new VisionIO() {
                                });
                                cameras = new Vision[]{limelight1};
                                swerve = new Swerve(new GyroIO() {
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
                NamedCommands.registerCommand("Cool example command", new InstantCommand(() -> swerve.resetPose(new Pose2d())));
                
                /*
                 * Auto Chooser
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                autoChooser.addOption("cool example auto", new SequentialCommandGroup());
                autoChooser.addDefaultOption("Do Nothing", Commands.none());
        }

        /**
         * Translational assist moves the xy position of the robot,
         * Rotational assist rotates it 
         */
        public void configureDriverAssists(){
                transAssist = new TranslationalAssist(
                        new Translation2d(), //where do we want to go to when the assist is active
                        () -> swerve.getPose().getTranslation(),  //where are we rn
                        () -> false // when this is true, the assist is active
                        );
                rotAssist = new RotationalAssist(
                        new Rotation2d(),
                        swerve::getYaw, 
                        () -> false
                        );
        }

        public void setAimAssists(Rotation2d rot, Translation2d trans){
                if(rot != null){
                        rotAssist.setDesiredRotation(rot);
                }
                if(trans != null){
                        transAssist.setDesiredTranslation(trans);
                }
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
         * 
         * Triggers wrap suppliers, and can be passed into the constructors,
         * just dont pass in the entire gamepad.
         * 
         * gp.button(1)
         * or
         * gp.x
         */
        public void configureBindings() {
                swerve.setDefaultCommand(
                   new TeleopSwerve(
                        swerve,
                        () -> translate.getRawAxis(0),
                        () -> -translate.getRawAxis(1), 
                        () -> -rotate.getRawAxis(0), 
                        () -> false,
                        () -> -rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.25 , 
                        () -> -translate.getRawAxis(Joystick.AxisType.kZ.value) * 0.25, 
                        transAssist, 
                        rotAssist
                        ));
                
                
                //sysid routines
                // gp.a.whileTrue(sysid.getSysidTest(SysidTest.QUASISTATIC_FORWARD));
                // gp.b.whileTrue(sysid.getSysidTest(SysidTest.QUASISTATIC_BACKWARD));
                // gp.x.whileTrue(sysid.getSysidTest(SysidTest.DYNAMIC_FORWARD));
                // gp.y.whileTrue(sysid.getSysidTest(SysidTest.DYNAMIC_BACKWARD));
                // gp.leftPaddle.whileTrue(swerve.zeroModules());
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public Pose3d getArmPoseAScope(){
                Rotation2d angle = Rotation2d.fromRotations(rotate.getRawAxis(2));
                return new Pose3d(0, -0.16, 0.23, new Rotation3d(angle.getRadians(), 0, 0));
        }
}
