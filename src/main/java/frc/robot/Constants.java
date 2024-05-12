package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.ModuleConstants;
import frc.lib.util.limelightConstants;


public class Constants {
    public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
            : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double loopPeriodSecs = 0.02; // 50 hz, default loop period
    // public static final Integer AmpConstants = 0;

    public static final class SwerveConstants {
        public static final String canivore = "CANt_open_file";

        public static final int pigeonID = 0;

        public static final double maxSpeed = 4.5; // max MODULE speed, NOT max chassis speed
        public static final double maxModuleSpeed = 5.5;
        public static final double maxAccel = 3;
        public static final double maxAngularVelocity = 2*Math.PI;
        public static final double maxAngularAcceleration = Math.PI / 2;

        public static final double stickDeadband = 0.02;

        public static final boolean useFOC = true;

        // AUTO ALIGNMENT ONLY !!!!!!11!!!1!1!!!
        public static final double rotationToleranceAlignment = 2.5;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.521; // 20.5 in -> meters
        public static final double wheelBase = 0.521; // meters
        public static final double driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase / 2, 2));
        public static final double wheelCircumference = Units.inchesToMeters(3.897375) * Math.PI; // avg of 3.8990
                                                                                                  // 3.8985 3.8925
                                                                                                  // 3.8995 checked
                                                                                                  // 2/10/2024 9:36:45
                                                                                                  // AM
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
        // public static final double driveGearRatio = (6.12 / 1.0);
        public static final double driveGearRatio = (5.36 / 1.0);

        /* Auto Constants */
        public static final double translationKP = 6;//7;//5 for outta the way?? idk;//7 before; //tuned for 4 m/s          //2.25; // tuned for .5 m/s
        public static final double translationKI = 0.0;
        public static final double translationKD = 1.4;

        public static final double rotationKP = 4;//4.5 //3 //4
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0; //0.4

        /* Teleop Constants */
        public static final double teleopRotationKP = 2;
        public static final double teleopRotationKI = 0.0;
        public static final double teleopRotationKD = 0.0;

        public static final double autoAlignRotationKP = 3.5;
        public static final double autoAlignRotationKI = 0.0;
        public static final double autoAlignRotationKD = 0.0;

        /* Module Translations */
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(moduleTranslations);

        /* Angle Motor Configs */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;

        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        public static final double angleKS = 0.0;
        public static final double angleKV = 0.0;
        public static final double angleKA = 0.0;

        public static final boolean angleSupplyCurrentLimitEnable = true;
        public static final double angleSupplyCurrentLimit = 25.0;
        public static final double angleSupplyCurrentThreshold = 40.0;
        public static final double angleSupplyTimeThreshold = 0.1;

        public static final boolean angleStatorCurrentLimitEnable = true;
        public static final double angleStatorCurrentLimit = 20;

        /* Drive Configs */
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        public static final double driveKS = 0.14267 / 12;// (0.15565 / 12);
        public static final double driveKV = 2.0718 / 12;// (2.0206 / 12);
        public static final double driveKA = 0.42622 / 12;// (0.94648 / 12);

        public static final boolean driveSupplyCurrentLimitEnable = true;
        public static final double driveSupplyCurrentLimit = 35.0;
        public static final double driveSupplyCurrentThreshold = 60.0;
        public static final double driveSupplyTimeThreshold = 0.1;

        public static final boolean driveStatorCurrentLimitEnable = true;
        public static final double driveStatorCurrentLimit = 60; //for now, maybe 50 if were feeling spicy

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* CanCoder Configs */
        public static final AbsoluteSensorRangeValue sensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        /*
         * Offsets must be done with bevels facing towards spivit motor
         */
        /* FRONT LEFT */
        public static final class Module0Constants {
            public static final int cancoderID = 1;
            public static final int angleMotorID = 2;
            public static final int driveMotorID = 1;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(285.46875); //284.94 //285.56 // 109.1 353.32

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* FRONT RIGHT */
        public static final class Module1Constants {
            public static final int cancoderID = 2;
            public static final int angleMotorID = 4;
            public static final int driveMotorID = 3;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(27.861328125); //27.42 214.1 9.14

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* BACK LEFT */
        public static final class Module2Constants {
            public static final int cancoderID = 3;
            public static final int angleMotorID = 6;
            public static final int driveMotorID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(176.30859375); //176.40 //176.57 // 203.1 234.66

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }
        

        /* BACK RIGHT */
        public static final class Module3Constants {
            public static final int cancoderID = 4;
            public static final int angleMotorID = 8;
            public static final int driveMotorID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(321.153);// 320.63 // 320.71 // 51.9 285.29

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        public static final Vector<N3> autostateStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1)); //formerly 0.1
        public static final Vector<N3> telestateStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(1));

    }

    public static final class FieldConstants {
        //put all season-specific field constants here

        public static final double fieldLength = 0; //meters
        public static final double fieldWidth = 0;
    }



    public static final class CANdleConstants{
        public static final int CANdleID = 0; 
        public static final int LedCount = 8; //without any led strips
    }

    public static final class LimelightConstants {

        //tags whose corresponding IDs are NOT in here will not be used in vision calculations
        //good for when a tag is wobbly or shifts over the course of a comp
        public static final int[] filtertags = {3,4,7,8,9,10,1,2,14,13};



        public static Matrix<N3, N1> trustautostdDev = VecBuilder.fill(0.2, 0.2, 9999999);
        public static Matrix<N3, N1> regautostdDev = VecBuilder.fill(0.9, 0.9, 9999999);

        public static Matrix<N3, N1> trusttelestdDev = VecBuilder.fill(0.2, 0.2, 9999999);
        public static Matrix<N3, N1> regtelestdDev = VecBuilder.fill(0.9, 0.9, 9999999);

        public static final double throwoutDist = 5.5; // meters, public static final double verticalFOV = 49.7; // degrees obviously
        public static final double rotationTolerance = 15; // maximum degrees that the limelight can be off before we throw out the pose
        public static final double poseAmbiguityThreshold = 0.4; //It's what Jonah uses
        public static final double zDistThreshold = 0.5; // meters that the limelight can be off the ground

        public static final class limelight1 {

            public static final String name = "limelight-1"; 
            public static final double verticalFOV = 49.7; // degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(0);
            public static final int detectorPiplineIndex = 0;
            public static final int apriltagPipelineIndex = 0;
            public static final int horPixles = 1280; //ll3
            public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));

            //TODO: boolean isDetector
            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, visionMeasurementStdDevs);
            public static final String ip = "10.1.77.11";

        }
    }
    
    public static final class PoseEstimatorConstants{
    }

}
