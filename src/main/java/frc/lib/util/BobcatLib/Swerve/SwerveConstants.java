package frc.lib.util.BobcatLib.Swerve;

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
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class SwerveConstants {

    public static final ReplanningConfig replanningConfig = new ReplanningConfig(false, false);
    public static final Rotation2d holoAlignTolerance = Rotation2d.fromDegrees(0);
    public static final double wheelBase = 0; //meters
    public static final double trackWidth = 0;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

    public static class Limits{
        public static class Chassis{
            public static final double maxSpeed = 0; // meters
            public static final double maxAccel = 0;
            public static final Rotation2d maxAngularVelocity = Rotation2d.fromRadians(0);
            public static final Rotation2d maxAngularAccel = Rotation2d.fromRadians(0);
        }
        public static class Module{
            public static final double maxSpeed = 0;
            public static final double maxAccel = 0;
            public static final double maxAngularVelocity = 0;
            public static final double maxAngularAccel = 0;
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
            public static final double rotKP = 0;
            public static final double rotKI = 0;
            public static final double rotKD = 0;
            public static final double transKP = 0;
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
    }

    public static class Odometry{
        //odometry stuff only, vision stuff is in LimelightConstants

        public static final Matrix<N3, N1> trustStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));
        public static final Matrix<N3, N1> distrustStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(1));
        public static final double odometryThrowoutAccel = 5.5; //m/s^2
        public static final double odometryDistrustAccel = 4; //m/s^2
    }
}
