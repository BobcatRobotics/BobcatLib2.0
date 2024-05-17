package frc.robot;



import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;


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
