package BobcatLib.Team177.Swerve;

import BobcatLib.Team254.ModuleConstants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class SwerveConstants {
  public static final String PIDConfigs = null;
  public ReplanningConfig replanningConfig = new ReplanningConfig();
  public Rotation2d holoAlignTolerance = Rotation2d.fromDegrees(0);
  public int pigeonID = 0;
  public KinematicsConstants kinematicsConstants;
  public SwerveSpeedLimits speedLimits;
  public PIDConfigs pidConfigs;
  public OdometryConstants odometryConstants;
  public SwerveModuleConfigs moduleConfigs;

  public SwerveConstants(
      ReplanningConfig replanningConfig,
      Rotation2d holoAlignTolerance,
      KinematicsConstants kinematicsConstants,
      SwerveSpeedLimits speedLimits,
      PIDConfigs pidConfigs,
      OdometryConstants odometryConstants,
      SwerveModuleConfigs moduleConfigs) {
    this.replanningConfig = replanningConfig;
    this.holoAlignTolerance = holoAlignTolerance;
    this.kinematicsConstants = kinematicsConstants;
    this.speedLimits = speedLimits;
    this.pidConfigs = pidConfigs;
    this.odometryConstants = odometryConstants;
    this.moduleConfigs = moduleConfigs;
  }

  public class KinematicsConstants {
    /**
     * All measurments in METERS!!!!!!!
     *
     * @param wheelBase distance between front and back wheels
     * @param trackWidth distance between left and right wheels
     * @param wheelCircumference you got this one
     */
    public KinematicsConstants(double wheelBase, double trackWidth, double wheelCircumference) {
      this.wheelBase = wheelBase;
      this.trackWidth = trackWidth;
      this.wheelCircumference = wheelCircumference;
      driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase / 2, 2));
      moduleTranslations =
          new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
          };
      kinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    public double wheelBase; // meters
    public double trackWidth;
    public double driveBaseRadius;
    public double wheelCircumference;
    public Translation2d[] moduleTranslations;
    public SwerveDriveKinematics kinematics;
  }

  public class SpeedLimit {
    public double maxVelocity;
    public double maxAccel;
    public Rotation2d maxAngularVelocity;
    public Rotation2d maxAngularAccel;

    /** linear mesurements in meters */
    public SpeedLimit(
        double maxVelocity,
        double maxAccel,
        Rotation2d maxAngularVelocity,
        Rotation2d maxAngularAccel) {
      this.maxVelocity = maxVelocity;
      this.maxAccel = maxAccel;
      this.maxAngularAccel = maxAngularAccel;
      this.maxAngularVelocity = maxAngularVelocity;
    }
  }

  public class SwerveSpeedLimits {
    public SpeedLimit chassisLimits;
    public SpeedLimit moduleLimits;

    /**
     * @param chassisLimits speed limits of the overall drivetrain
     * @param moduleLimits speed limits of each individual module
     */
    public SwerveSpeedLimits(SpeedLimit chassisLimits, SpeedLimit moduleLimits) {
      this.chassisLimits = chassisLimits;
      this.moduleLimits = moduleLimits;
    }
  }

  public class SwervePIDConfig {
    public double rotKP;
    public double rotKI;
    public double rotKD;
    public double transKP;
    public double transKI;
    public double transKD;
    public PIDConstants transPidConstants;
    public PIDConstants rotPidConstants;

    public SwervePIDConfig(
        double rotKP, double rotKI, double rotKD, double transKP, double transKI, double transKD) {
      this.rotKP = rotKP;
      this.rotKI = rotKI;
      this.rotKD = rotKD;
      this.transKP = transKP;
      this.transKI = transKI;
      this.transKD = transKD;
      transPidConstants = new PIDConstants(transKP, transKI, transKD);
      rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
    }
  }

  public class SwerveMotorConfig {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public InvertedValue motorInvert;
    public NeutralModeValue neutralMode;
    public boolean supplyCurrentLimitEnable;
    public double supplyCurrentLimit;
    public double supplyCurrentThreshold;
    public double supplyTimeThreshold;
    public boolean statorCurrentLimitEnable;
    public double statorCurrentLimit;
    public double openLoopRamp;
    public double closedLoopRamp;

    /**
     * see docs on current limiting for more info
     * https://www.chiefdelphi.com/t/current-limiting-on-swerve/454392
     */
    public SwerveMotorConfig(
        double rotKP,
        double rotKI,
        double rotKD,
        double transKP,
        double transKI,
        double transKD,
        InvertedValue motorInvert,
        NeutralModeValue neutralMode,
        boolean supplyCurrentLimitEnable,
        double supplyCurrentLimit,
        double supplyCurrentThreshold,
        double supplyTimeThreshold,
        boolean statorCurrentLimitEnable,
        double statorCurrentLimit,
        double openLoopRamp,
        double closedLoopRamp) {
      this.kP = rotKP;
      this.kI = rotKI;
      this.kD = rotKD;
      this.kS = transKP;
      this.kV = transKI;
      this.kA = transKD;
      this.motorInvert = motorInvert;
      this.neutralMode = neutralMode;
      this.supplyCurrentLimitEnable = supplyCurrentLimitEnable;
      this.supplyCurrentLimit = supplyCurrentLimit;
      this.supplyCurrentThreshold = supplyCurrentThreshold;
      this.supplyTimeThreshold = supplyTimeThreshold;
      this.statorCurrentLimitEnable = statorCurrentLimitEnable;
      this.statorCurrentLimit = statorCurrentLimit;
      this.openLoopRamp = openLoopRamp;
      this.closedLoopRamp = closedLoopRamp;
    }
  }

  public class PIDConfigs {
    public SwervePIDConfig autoConfig;
    public SwervePIDConfig teleopConfig;
    public SwervePIDConfig autoAlignConfig;
    public SwerveMotorConfig angleMotorConfig;
    public SwerveMotorConfig driveMotorConfig;

    /**
     * gains taken from sysid should likely need to be devided by 12 to convert from volts to
     * percent
     *
     * @param autoConfig rotation and translation gains to use in auto
     * @param teleopConfig rotation and translation gains to use in teleop
     * @param autoAlignConfig rotation gains to use in auto, translation gains will not be used and
     *     can be set to any number
     */
    public PIDConfigs(
        SwervePIDConfig autoConfig,
        SwervePIDConfig teleopConfig,
        SwervePIDConfig autoAlignConfig,
        SwerveMotorConfig angleMotorConfig,
        SwerveMotorConfig driveMotorConfig) {
      this.autoConfig = autoConfig;
      this.teleopConfig = teleopConfig;
      this.autoAlignConfig = autoAlignConfig;
      this.angleMotorConfig = angleMotorConfig;
      this.driveMotorConfig = driveMotorConfig;
    }
  }

  public class OdometryConstants {
    // odometry stuff only, vision stuff is in LimelightConstants
    /**
     * units based in meters
     *
     * @param trustStdDevs std devs to use when we trust the measurements
     * @param distrustStdDevs std devs to use when the measurements are unreliable
     * @param odometryThrowoutAccel acceleration beyond this value will throw out odometry
     *     measurements
     * @param odometryDistrustAccel aceceleration beyond this value will use distrust measurements
     */
    public OdometryConstants(
        Matrix<N3, N1> trustStdDevs,
        Matrix<N3, N1> distrustStdDevs,
        double odometryThrowoutAccel,
        double odometryDistrustAccel) {
      this.trustStdDevs = trustStdDevs;
      this.distrustStdDevs = distrustStdDevs;
      this.odometryDistrustAccel = odometryDistrustAccel;
      this.odometryThrowoutAccel = odometryThrowoutAccel;
    }

    public Matrix<N3, N1> trustStdDevs;
    public Matrix<N3, N1> distrustStdDevs;
    public double odometryThrowoutAccel; // m/s^2
    public double odometryDistrustAccel; // m/s^2
  }

  public class ModuleConfig {
    public int cancoderID;
    public int angleMotorID;
    public int driveMotorID;
    public Rotation2d angleOffset;
    public ModuleConstants moduleConstants;

    public ModuleConfig(
        int cancoderID, int angleMotorID, int driveMotorID, Rotation2d angleOffset) {
      this.cancoderID = cancoderID;
      this.angleMotorID = angleMotorID;
      this.driveMotorID = driveMotorID;
      this.angleOffset = angleOffset;
      this.moduleConstants =
          new ModuleConstants(driveMotorID, angleMotorID, cancoderID, angleOffset);
    }
  }

  public static class SwerveModuleConfigs {
    ModuleConfig frontLeft;
    ModuleConfig frontRight;
    ModuleConfig backLeft;
    ModuleConfig backRight;

    public SwerveModuleConfigs(
        ModuleConfig frontLeft,
        ModuleConfig frontRight,
        ModuleConfig backLeft,
        ModuleConfig backRight) {
      this.frontLeft = frontLeft;
      this.frontRight = frontRight;
      this.backLeft = backLeft;
      this.backRight = backRight;
    }
  }
}
