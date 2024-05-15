package frc.robot.Subsystems.Superstructure.Arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;

public class ArmIOFalcon implements ArmIO{
    private TalonFX armMotor;
    private CANcoder armEncoder;
    private StatusSignal<Double> position;
    private StatusSignal<Double> canPosition;
    private StatusSignal<Double> statorCurrent;
    private Rotation2d desiredPose = new Rotation2d();
    private final PositionTorqueCurrentFOC positionTorqueControl = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);


    public ArmIOFalcon(){
        armEncoder = new CANcoder(ArmConstants.encoderID);
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        armEncoder.getConfigurator().apply(cancoderConfigs);
        cancoderConfigs.MagnetSensor.SensorDirection = ArmConstants.sensorDirection;
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = ArmConstants.sensorRangeValue;
        cancoderConfigs.MagnetSensor.MagnetOffset = ArmConstants.magnetOffset.getRotations();
        
        armEncoder.getConfigurator().apply(cancoderConfigs);

        armMotor = new TalonFX(ArmConstants.motorID);
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        armMotor.getConfigurator().apply(motorConfigs);
        motorConfigs.Slot0.kP = ArmConstants.kp;
        motorConfigs.Slot0.kI = ArmConstants.ki;
        motorConfigs.Slot0.kD = ArmConstants.kd;
        motorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // 0 must be horizontal, sensor must be one-to-one with mechanism
        motorConfigs.Feedback.FeedbackRemoteSensorID = ArmConstants.encoderID;
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; 
        armMotor.getConfigurator().apply(motorConfigs);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.angle = Rotation2d.fromRotations(armEncoder.getAbsolutePosition().getValueAsDouble());
        inputs.desiredAngle = desiredPose;
    }

    @Override
    public void setAngle(Rotation2d angle){
        desiredPose = angle;
        armMotor.setControl(
            positionTorqueControl
            .withPosition(angle.getRotations())
            .withFeedForward(ArmConstants.arbitraryFF)
            );
    }

}
