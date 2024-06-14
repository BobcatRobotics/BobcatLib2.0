package frc.lib.BobcatLib.Swerve;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.core.JsonParser;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.BobcatLib.Team254.ModuleConstants;

public class ParsedSwerveConstants {
    
    public final String CANbus = "CANt_open_file"; // if using canivore, put its name here (177 names it's canivores
                                                   // 'CANt_open_file'),
                                                   // otherwise, put an empty string (""), and the default rio canbus
                                                   // will be used
    public final ReplanningConfig replanningConfig = new ReplanningConfig(false, false);
    public final Rotation2d holoAlignTolerance = Rotation2d.fromDegrees(0);
    public final int pigeonID = 1;
    public final boolean useFOC = true;
    public final double angleGearRatio = ((150.0 / 7.0) / 1.0);
    public final double driveGearRatio = (6.12 / 1.0);

    public class Kinematics {
        public final double wheelBase = 0.521; // meters
        public final double trackWidth = 0.521;
        public final double driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase / 2, 2)); // TODO assumes square
                                                                                         // wheelbase
        public final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
        public final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };
        public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    public class Limits {
        public class Chassis {
            public final double maxSpeed = 4.5; // meters
            public final double maxAccel = 3;
            public final Rotation2d maxAngularVelocity = Rotation2d.fromDegrees(360 * 2.5);
            public final Rotation2d maxAngularAccel = Rotation2d.fromDegrees(360 / 2);
        }

        public class Module {
            public final double maxSpeed = 5.5;
            public final double maxAccel = 3;
            public final Rotation2d maxAngularVelocity = Rotation2d.fromDegrees(360 * 2.5);
            public final Rotation2d maxAngularAccel = Rotation2d.fromDegrees(360 * 2.5);
        }
    }

    /**
     * also contains gains
     */
    public class Configs {
        public class Auto {
            public final double rotKP = 4;
            public final double rotKI = 0;
            public final double rotKD = 0;
            public final double transKP = 6;
            public final double transKI = 0;
            public final double transKD = 1.4;
            public final PIDConstants transPidConstants = new PIDConstants(transKP, transKI, transKD);
            public final PIDConstants rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
        }

        public class Teleop {
            public final double rotKP = 2;
            public final double rotKI = 0;
            public final double rotKD = 0;
            public final double transKP = 10;
            public final double transKI = 0;
            public final double transKD = 0;
            public final PIDConstants transPidConstants = new PIDConstants(transKP, transKI, transKD);
            public final PIDConstants rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
        }

        public class AutoAlign {
            public final Rotation2d tolerance = Rotation2d.fromDegrees(2);
            public final double rotationKP = 3.5;
            public final double rotationKI = 0;
            public final double rotationKD = 0;
            public final PIDConstants rotPidConstants = new PIDConstants(rotationKP, rotationKI, rotationKD);
        }

        public class Module {
            public class Angle {
                public final double kP = 0.3;
                public final double kI = 0;
                public final double kD = 0;
                public final double kS = 0;
                public final double kV = 0;
                public final double kA = 0;
                public final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
                public final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
                public final boolean angleSupplyCurrentLimitEnable = true;
                public final double angleSupplyCurrentLimit = 25.0;
                public final double angleSupplyCurrentThreshold = 40.0;
                public final double angleSupplyTimeThreshold = 0.1;
                public final boolean angleStatorCurrentLimitEnable = true;
                public final double angleStatorCurrentLimit = 20;
            }

            public class Drive {
                // sysid bases its calculations off of voltage, so we convert to percent.
                // gain in volts * (1.00 (100%) / 12 volts per battery) = gain
                public final double kS = 0.088235 / 12;
                public final double kA = 0.041076 / 12;
                public final double kV = 2.2301 / 12;
                public final double kP = 0.02;
                public final double kI = 0;
                public final double kD = 0;
                public final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
                public final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
                public final boolean driveSupplyCurrentLimitEnable = true;
                public final double driveSupplyCurrentLimit = 35.0;
                public final double driveSupplyCurrentThreshold = 60.0;
                public final double driveSupplyTimeThreshold = 0.1;
                public final boolean driveStatorCurrentLimitEnable = true;
                public final double driveStatorCurrentLimit = 60;
                public final double openLoopRamp = 0.25;
                public final double closedLoopRamp = 0.0;
            }

            public class CANCoder {
                public final AbsoluteSensorRangeValue sensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
                public final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            }

        }
    }

    public class Odometry {
        // odometry stuff only, vision stuff is in LimelightConstants

        public final Matrix<N3, N1> trustStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));
        public final Matrix<N3, N1> distrustStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(1));
        public final double odometryThrowoutAccel = 5.5; // m/s^2
        public final double odometryDistrustAccel = 4; // m/s^2
    }

    public class Module {
        /*
         * Offsets must be done with bevels facing towards spivit motor
         */
        /* FRONT LEFT */
        public final class Module0Constants {
            public final int cancoderID = 1;
            public final int angleMotorID = 2;
            public final int driveMotorID = 1;

            public final Rotation2d angleOffset = Rotation2d.fromDegrees(353.3); // round to tenths

            public final String name = "front-left";

            public final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset, name);
        }

        /* FRONT RIGHT */
        public final class Module1Constants {
            public final int cancoderID = 2;
            public final int angleMotorID = 4;
            public final int driveMotorID = 3;
            public final String name = "front-right";

            public final Rotation2d angleOffset = Rotation2d.fromDegrees(188.7);
            public final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset, name);
        }

        /* BACK LEFT */
        public final class Module2Constants {
            public final int cancoderID = 3;
            public final int angleMotorID = 6;
            public final int driveMotorID = 5;
            public final Rotation2d angleOffset = Rotation2d.fromDegrees(54.0);
            public final String name = "back-left";

            public final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset, name);
        }

        /* BACK RIGHT */
        public final class Module3Constants {
            public final int cancoderID = 4;
            public final int angleMotorID = 8;
            public final int driveMotorID = 7;
            public final Rotation2d angleOffset = Rotation2d.fromDegrees(105.0);
            public final String name = "back-right";

            public final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset, name);
        }
    }

    public class AimAssistConstants {
        public final double kP_X = 0.2; // these will be the same value in most cases
        public final double kP_Y = 0.2;
        public final double kP_Theta = 1;
    }
}

// TODO deadband docs, add to gamepad
