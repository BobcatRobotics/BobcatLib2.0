package frc.lib.BobcatLib.Swerve.SwerveModule;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.BobcatLib.Annotations.SeasonBase;
import frc.lib.BobcatLib.Swerve.PhoenixOdometryThread;
import frc.lib.Team254.ModuleConstants;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.lib.BobcatLib.Swerve.SwerveConstants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

@SeasonBase
public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final Rotation2d encoderOffset;

    private final DutyCycleOut driveRequest;
    private final DutyCycleOut angleRequest;
    private final VoltageOut driveVoltageRequest;

    private final StatusSignal<Double> internalTempDrive;
    private final StatusSignal<Double> internalTempAngle;
    private final StatusSignal<Double> driveAppliedVolts;

    private final Queue<Double> timestampQueue;

    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAcceleration;

    private final Queue<Double> anglePositionQueue;
    private final StatusSignal<Double> angleAbsolutePosition;
    private final String name;

    public SwerveModuleIOFalcon(ModuleConstants moduleConstants) {
        name = moduleConstants.moduleName;
        encoderOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.canivore);
        configAngleEncoder();
        angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.canivore);
        configAngleMotor();
        driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.canivore);
        configDriveMotor();

        driveRequest = new DutyCycleOut(0.0).withEnableFOC(SwerveConstants.useFOC);
        angleRequest = new DutyCycleOut(0.0).withEnableFOC(SwerveConstants.useFOC);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(SwerveConstants.useFOC);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();
        angleAbsolutePosition = angleEncoder.getAbsolutePosition();
        anglePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(angleEncoder, angleEncoder.getPosition());

        internalTempDrive = driveMotor.getDeviceTemp();
        internalTempAngle = angleMotor.getDeviceTemp();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, drivePosition, driveVelocity, angleAbsolutePosition, driveAppliedVolts);
        BaseStatusSignal.setUpdateFrequencyForAll(5, internalTempAngle, internalTempDrive);
        driveMotor.optimizeBusUtilization();
        angleMotor.optimizeBusUtilization();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.offset = encoderOffset;
        BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        angleAbsolutePosition,
        internalTempAngle,
        internalTempDrive,
        driveAcceleration,
        driveAppliedVolts);


        inputs.wheelPositionRot = drivePosition.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.wheelVelocityRotPerSec = driveVelocity.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.wheelAcceleration = driveAcceleration.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.canCoderPositionRot = Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).minus(encoderOffset).getRadians())).getRotations();
        inputs.rawCanCoderPositionDeg = Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).getDegrees(); // Used only for shuffleboard to display values to get offsets

        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / SwerveConstants.driveGearRatio)
                .toArray();
        inputs.odometryAnglePositions =
            anglePositionQueue.stream()
                // .map((Double value) -> Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(value).minus(encoderOffset).getRadians())))
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        anglePositionQueue.clear();

        inputs.internalTempAngle = internalTempAngle.getValueAsDouble();
        inputs.internalTempDrive = internalTempDrive.getValueAsDouble();
        inputs.appliedDriveVoltage = driveAppliedVolts.getValueAsDouble();
    }
    
    @Override
    public String getModule(){
        return name;
    }
    

    /**
     * Sets the percent out of the drive motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setDrivePercentOut(double percent) {
        driveMotor.setControl(driveRequest.withOutput(percent));
    }

    /**
     * Stops the drive motor
     */
    public void stopDrive() {
        driveMotor.stopMotor();
    }
    
    @Override
    public void setVolts(Measure<Voltage> volts){
        driveMotor.setControl(driveVoltageRequest.withOutput(volts.magnitude()));
    }
    /**
     * Sets the neutral mode of the drive motor
     * @param mode mode to set it to
     */
    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    /**
     * Sets the percent out of the angle motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setAnglePercentOut(double percent) {
        angleMotor.setControl(angleRequest.withOutput(percent));
    }

    /**
     * Stops the angle motor
     */
    public void stopAngle() {
        angleMotor.stopMotor();
    }

    /**
     * Sets the neutral mode of the angle motor
     * @param mode mode to set it to
     */
    public void setAngleNeutralMode(NeutralModeValue mode) {
        angleMotor.setNeutralMode(mode);
    }

    /**
     * Applies all configurations to the drive motor
     */
    public void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.Configs.Module.Drive.driveSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.Configs.Module.Drive.driveSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.Configs.Module.Drive.driveSupplyCurrentThreshold;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.Configs.Module.Drive.driveSupplyTimeThreshold;

        config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.Configs.Module.Drive.driveStatorCurrentLimitEnable;
        config.CurrentLimits.StatorCurrentLimit = SwerveConstants.Configs.Module.Drive.driveStatorCurrentLimit;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.Configs.Module.Drive.openLoopRamp;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = SwerveConstants.Configs.Module.Drive.openLoopRamp;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.Configs.Module.Drive.openLoopRamp;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.Configs.Module.Drive.closedLoopRamp;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = SwerveConstants.Configs.Module.Drive.closedLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.Configs.Module.Drive.closedLoopRamp;

        

        config.MotorOutput.Inverted = SwerveConstants.Configs.Module.Drive.driveMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.Configs.Module.Drive.driveNeutralMode;

        driveMotor.getConfigurator().apply(config);

        driveMotor.getConfigurator().setPosition(0);
    }

    /**
     * Applies all configurations to the angle motor
     */
    public void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.Configs.Module.Angle.angleSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.Configs.Module.Angle.angleSupplyCurrentThreshold;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.Configs.Module.Angle.angleSupplyTimeThreshold;

        config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.Configs.Module.Angle.angleStatorCurrentLimitEnable;
        config.CurrentLimits.StatorCurrentLimit = SwerveConstants.Configs.Module.Angle.angleStatorCurrentLimit;
        

        config.MotorOutput.Inverted = SwerveConstants.Configs.Module.Angle.angleMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.Configs.Module.Angle.angleNeutralMode;

        angleMotor.getConfigurator().apply(config);
    }


    /**
     * Applies all configurations to the angle absolute encoder
     */
    public void configAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

        config.MagnetSensor.AbsoluteSensorRange = SwerveConstants.Configs.Module.CANCoder.sensorRange;
        config.MagnetSensor.SensorDirection = SwerveConstants.Configs.Module.CANCoder.sensorDirection;

        angleEncoder.getConfigurator().apply(config);
    }


}