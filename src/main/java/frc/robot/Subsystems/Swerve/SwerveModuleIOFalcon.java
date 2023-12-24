package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final Rotation2d angleOffset;

    public SwerveModuleIOFalcon(ModuleConstants moduleConstants) {
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, SwerveConstants.canivore);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID, SwerveConstants.canivore);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID, SwerveConstants.canivore);
        configDriveMotor();

        this.angleOffset = moduleConstants.angleOffset;
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePosition = driveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocity = driveMotor.getVelocity().getValueAsDouble();

        inputs.anglePosition = angleMotor.getPosition().getValueAsDouble();

        inputs.canCoderPosition = angleEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    /* ----- Drive motor methods ----- */
    private void configDriveMotor(){        
        TalonFXConfiguration config = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(config);

        config.Slot0.kP = SwerveConstants.driveKP;
        config.Slot0.kI = SwerveConstants.driveKI;
        config.Slot0.kD = SwerveConstants.driveKD;

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.driveSupplyCurrentThreshold;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.driveSupplyTimeThreshold;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;

        config.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;
        driveMotor.getConfigurator().apply(config);
        driveMotor.setControl(new PositionDutyCycle(0).withEnableFOC(SwerveConstants.useFOC).withSlot(0));
    }

    public void setDrive(ControlRequest request) {
        driveMotor.setControl(request);
    }

    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    /* ----- Angle motor methods ----- */
    private void configAngleMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(config);

        config.Slot0.kP = SwerveConstants.angleKP;
        config.Slot0.kI = SwerveConstants.angleKI;
        config.Slot0.kD = SwerveConstants.angleKD;

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.angleSupplyCurrentThreshold;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.angleSupplyTimeThreshold;

        config.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.angleNeutralMode;

        angleMotor.getConfigurator().apply(config);
    }

    public void setAngle(ControlRequest request) {
        angleMotor.setControl(request);
    }

    public void setAngleNeutralMode(NeutralModeValue mode) {
        angleMotor.setNeutralMode(mode);
    }

    /* ----- Angle encoder methods ----- */
    private void configAngleEncoder(){        
        CANcoderConfiguration config = new CANcoderConfiguration();
        angleEncoder.getConfigurator().apply(config);

        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoder.getConfigurator().apply(config);
    }

    public Rotation2d getModuleOffset() {
        return angleOffset;
    }
}
