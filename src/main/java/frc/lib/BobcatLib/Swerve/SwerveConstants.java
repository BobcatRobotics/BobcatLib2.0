package frc.lib.BobcatLib.Swerve;

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


public class SwerveConstants {
    public static final double stickDeadband = 0.005;
    public static final ReplanningConfig replanningConfig = new ReplanningConfig(false, false);
    public static final Rotation2d holoAlignTolerance = Rotation2d.fromDegrees(0);
    public static final double wheelBase = 0.65; //meters
    public static final double trackWidth = 0.65;
    public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

    public static class Limits{
        public static class Chassis{
            public static final double maxSpeed = 10; // meters
            public static final double maxAccel = 10;
            public static final Rotation2d maxAngularVelocity = Rotation2d.fromDegrees(360*2.5);
            public static final Rotation2d maxAngularAccel = Rotation2d.fromDegrees(360*2);
        }
        public static class Module{
            public static final double maxSpeed = 10;
            public static final double maxAccel = 10;
            public static final double maxAngularVelocity = 5;
            public static final double maxAngularAccel = 5;
        }
    }


    public static class Gains{
        public static class Auto{
            public static final double rotKP = 0;
            public static final double rotKI = 0;
            public static final double rotKD = 0;
            public static final double transKP = 0;
            public static final double transKI = 0;
            public static final double transKD = 0;
            public static final PIDConstants transPidConstants = new PIDConstants(transKP, transKI, transKD);
            public static final PIDConstants rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
        }
        public static class Teleop{
            public static final double rotKP = 10;
            public static final double rotKI = 0;
            public static final double rotKD = 0;
            public static final double transKP = 10;
            public static final double transKI = 0;
            public static final double transKD = 0;
            public static final PIDConstants transPidConstants = new PIDConstants(transKP, transKI, transKD);
            public static final PIDConstants rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
        }
        public static class AutoAlign{
            public static final double rotationKP = 0;
            public static final double rotationKI = 0;
            public static final double rotationKD = 0;
            public static final PIDConstants rotPidConstants = new PIDConstants(rotationKP, rotationKI, rotationKD);
        }
        public static class Module{
            public static final double kS = 0.14267 / 12;
            public static final double kA = 2.0718 / 12;
            public static final double kV = 0.42622 / 12;
            public static final double kDriveP = 0.1;
            public static final double kDriveI = 0;
            public static final double kDriveD = 0;
            public static final double kRotP = 0.3;
            public static final double kRotI = 0;
            public static final double kRotD = 0;


        }
    }

    public static class Odometry{
        //odometry stuff only, vision stuff is in LimelightConstants

        public static final Matrix<N3, N1> trustStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));
        public static final Matrix<N3, N1> distrustStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(1));
        public static final double odometryThrowoutAccel = 5.5; //m/s^2
        public static final double odometryDistrustAccel = 4; //m/s^2
    }
}
