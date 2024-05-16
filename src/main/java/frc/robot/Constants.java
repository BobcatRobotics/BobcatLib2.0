package frc.robot;



import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.BobcatLib.Vision.limelightConstants;


public class Constants {
    public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
            : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);
    public static final boolean isTuningMode = DriverStation.isTest();

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

    public static final String canivore = "CANt_open_file";



    public static final class FieldConstants {
        //put all season-specific field constants here

        public static final double fieldLength = 0; //meters
        public static final double fieldWidth = 0;
    }



    public static final class CANdleConstants{
        public static final int CANdleID = 0; 
        public static final int LedCount = 8; //without any led strips
    }

    public static final class VisionConstants {

        //tags whose corresponding IDs are NOT in here will not be used in vision calculations
        //good for when a tag is wobbly or shifts over the course of a comp
        public static final int[] filtertags = {3,4,7,8,9,10,1,2,14,13};



        public static Matrix<N3, N1> trustautostdDev = VecBuilder.fill(0.2, 0.2, 9999999);
        public static Matrix<N3, N1> regautostdDev = VecBuilder.fill(0.9, 0.9, 9999999);

        public static Matrix<N3, N1> trusttelestdDev = VecBuilder.fill(0.2, 0.2, 9999999);
        public static Matrix<N3, N1> regtelestdDev = VecBuilder.fill(0.9, 0.9, 9999999);

        public static final double throwoutDist = 5.5; // meters, public static final double verticalFOV = 49.7; // degrees obviously
        public static final double rotationTolerance = 15; // maximum degrees that the limelight can be off before we throw out the pose
        public static final double zDistThreshold = 0.5; // meters that the limelight can be off the ground

        public static final class limelight1 {

            public static final String name = "limelight-1"; 
            public static final double verticalFOV = 49.7; // degrees obviously
            public static final double horizontalFOV = 63.3;
            public static final double limelightMountHeight = Units.inchesToMeters(0);
            public static final int detectorPiplineIndex = 0;
            public static final int apriltagPipelineIndex = 0;
            public static final int horPixles = 1280; //ll3
            public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1,
                    Units.degreesToRadians(10));

            public static final limelightConstants constants = new limelightConstants(name, verticalFOV, horizontalFOV,
                    limelightMountHeight, detectorPiplineIndex, apriltagPipelineIndex, horPixles, visionMeasurementStdDevs);
            public static final String ip = "10.1.77.11";

        }
    }
    
    public static final class PoseEstimatorConstants{
    }



    public static final class ArmConstants{
        public static final double lengthMeters = 0.5;
        public static final int encoderID = 1;
        public static final int motorID = 10;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final AbsoluteSensorRangeValue sensorRangeValue = AbsoluteSensorRangeValue.Unsigned_0To1; 
        public static final Rotation2d magnetOffset = Rotation2d.fromDegrees(0); //TODO 0 degrees must be horizontal! 
        public static final double kp = 0.275;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double arbitraryFF = 0;
    }

    public static final class ElevatorConstants{
        public static final int motorID = 9;
        public static final int bottomLimit = 0;
        public static final int topLimit = -236710; //maybe?
    }

}
