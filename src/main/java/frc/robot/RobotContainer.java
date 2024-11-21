// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import BobcatLib.Team177.Gamepads.Logitech;
import BobcatLib.Team177.Swerve.Constants.SwerveConstantCreator;
import BobcatLib.Team177.Swerve.Constants.SwerveConstants;
import BobcatLib.Team177.Swerve.Gyro.GyroIOPigeon2;
import BobcatLib.Team177.Swerve.SwerveModule.SwerveModuleIOFalcon;
import BobcatLib.Team177.Vision.Vision;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import smile.math.matrix.fp32.Matrix;
import BobcatLib.Team177.Swerve.SwerveBase;



public class RobotContainer {

        /* Joysticks + Gamepad */
        private final Logitech gp = new Logitech(0);

        /* Subsystems */
        public final SwerveBase swerve;
        public SwerveConstants swerveConstants;
        // public Vision limelight1;
        // public Vision[] cameras;

        /* Commands */

        /* Shuffleboard Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                try {
                        swerveConstants = SwerveConstantCreator.parseConstants(null);
                } catch (Exception e) {
                        // TODO: handle exception
                }
                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        default: 
                                swerve = new SwerveBase(swerveConstants, new int[]{}, new Matrix[]{}, null);


                }
                configureBindings();
        }

        public boolean autoChooserInitialized() {
                return autoChooser.get() != null;
        }

        /**
         * this should only be called once DS and FMS are attached
         */
        public void configureAutos() {

                /*
                 * Auto Events
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                // NamedCommands.registerCommand("PathfindingCommand", swerve.driveToPose(new
                // Pose2d()));

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
                swerve.setAimAssistTranslation(new Translation2d(5, 5));
                TranslationAssist transAssist = new TranslationAssist(() -> swerve.getAimAssistTranslation(),
                                () -> swerve.getPose().getTranslation(), gp.a, () -> false);
                RotationalAssist rotAssist = new RotationalAssist(() -> new Rotation2d(), () -> swerve.getYaw(),
                                () -> false, () -> false);

                swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                swerve,
                                                gp.leftYAxis,
                                                gp.leftXAxis,
                                                gp.rightXAxis,
                                                gp.rb,
                                                () -> 0,
                                                () -> 0,
                                                transAssist,
                                                rotAssist));

                // sysid routines
                gp.start.onTrue(new InstantCommand(() -> swerve.resetPose(new Pose2d())));
                gp.back.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        // public Pose3d getArmPoseAScope(){
        // Rotation2d angle = Rotation2d.fromRotations(rotate.getRawAxis(2));
        // return new Pose3d(0, -0.16, 0.23, new Rotation3d(angle.getRadians(), 0, 0));
        // }
}
