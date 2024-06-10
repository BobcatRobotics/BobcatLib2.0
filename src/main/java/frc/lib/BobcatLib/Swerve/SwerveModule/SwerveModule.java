package frc.lib.BobcatLib.Swerve.SwerveModule;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import frc.lib.BobcatLib.Annotations.SeasonBase;
import frc.lib.BobcatLib.Swerve.SwerveConstants;
import frc.lib.BobcatLib.Swerve.SwerveConstants.Configs;
import frc.lib.BobcatLib.Swerve.SwerveConstants.Limits;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

@SeasonBase
public class SwerveModule {
    public enum SysidTest {
        QUASISTATIC_FORWARD,
        QUASISTATIC_BACKWARD,
        DYNAMIC_FORWARD,
        DYNAMIC_BACKWARD
    }
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public final int index;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Configs.Module.Drive.kS,
            Configs.Module.Drive.kV, Configs.Module.Drive.kA);
    private PIDController driveController = new PIDController(Configs.Module.Drive.kP, Configs.Module.Drive.kI,
            Configs.Module.Drive.kD);
    private PIDController angleController = new PIDController(Configs.Module.Angle.kP, Configs.Module.Angle.kI,
            Configs.Module.Angle.kD);

    private SwerveModuleState desiredState = new SwerveModuleState();

    private Rotation2d lastAngle;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        angleController.enableContinuousInput(0, 2 * Math.PI);

        lastAngle = getState().angle;
        
      
    }

    public void sysidLog(SysIdRoutineLog log){
        log.motor(io.getModule())
        .linearPosition(m_distance.mut_replace(getPositionMeters(), Meters))
        .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSec(), MetersPerSecond))
        .voltage(m_appliedVoltage.mut_replace(getVoltage(), Volts));
            
    }

    public void setVoltage(Measure<Voltage> volts){
        setDesiredAngle(new Rotation2d());
        io.setVolts(volts);
    }

 

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i]
                    * (SwerveConstants.Kinematics.wheelCircumference / (2 * Math.PI));
            Rotation2d angle = inputs.odometryAnglePositions[i].minus(
                    inputs.offset != null ? inputs.offset : new Rotation2d());
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /**
     * Sets the swerve module to the desired state
     * 
     * @param state the desired state of the swerve module
     * @return the optimized swerve module state that it was set to
     */
    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        // if we are running at under 1% of max speed, dont change the angle, minimizes
        // module jitter
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Limits.Module.maxSpeed * 0.01))
                ? lastAngle
                : optimizedState.angle;

        // It is important that we use radians for the PID
        // so we can update the drive speed as shown below
        double output = MathUtil.clamp(
                angleController.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()), -1.0, 1.0);
        io.setAnglePercentOut(output);

        // Update velocity based on turn error
        optimizedState.speedMetersPerSecond *= Math.cos(angleController.getPositionError());

        double velocity = optimizedState.speedMetersPerSecond / SwerveConstants.Kinematics.wheelCircumference;
        double velocityOut = MathUtil.clamp(
                driveController.calculate(inputs.wheelVelocityRotPerSec, velocity)
                        + driveFeedforward.calculate(velocity),
                -1.0, 1.0);
        if (velocity == 0) {
            velocityOut = 0;
        }
        io.setDrivePercentOut(velocityOut);

        desiredState = optimizedState;
        lastAngle = angle;
        return optimizedState;
    }

    public void setDesiredAngle(Rotation2d ang) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(new SwerveModuleState(0, ang), getAngle());

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Limits.Module.maxSpeed * 0.01))
                ? lastAngle
                : optimizedState.angle;

        // It is important that we use radians for the PID
        // so we can update the drive speed as shown below
        double output = MathUtil.clamp(
                angleController.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()), -1.0, 1.0);
        io.setAnglePercentOut(output);
        lastAngle = angle;
    }

    /**
     * Stops the drive and angle motors
     */
    public void stop() {
        io.stopAngle();
        io.stopDrive();
    }

    /**
     * Sets the neutral mode of the angle motor
     * 
     * @param mode the mode to set it to
     */
    public void setAngleNeutralMode(NeutralModeValue mode) {
        io.setAngleNeutralMode(mode);
    }

    /**
     * Sets the neutral mode of the drive motor
     * 
     * @param mode the mode to set it to
     */
    public void setDriveNeutralMode(NeutralModeValue mode) {
        io.setDriveNeutralMode(mode);
    }

    /**
     * Gets the current angle of the swerve module from the CANcoder
     * 
     * @return the angle of the module
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.canCoderPositionRot);
    }

    /**
     * Gets the current position of the drive motor in meters
     * 
     * @return drive motor position, in meters
     */
    public double getPositionMeters() {
        return inputs.wheelPositionRot * SwerveConstants.Kinematics.wheelCircumference;
    }

    /**
     * Gets the current velocity of the drive motor in meters per second
     * 
     * @return velocity, in meter per second
     */
    public double getVelocityMetersPerSec() {
        return inputs.wheelVelocityRotPerSec * SwerveConstants.Kinematics.wheelCircumference;
    }

    /**
     * Gets the current position of the swerve module
     * 
     * @return the swerve module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Gets the current state of the swerve module
     * 
     * @return the swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Gets the current desired state that the swerve module has been set to
     * 
     * @return the desired state
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Gets the raw value of the CANcoder, before the offset is applied. Used only
     * for SmartDashboard
     * 
     * @return CANcoder position, in degrees
     */
    public double getRawCanCoder() {
        return inputs.rawCanCoderPositionDeg;
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getDriveAcceleration() {
        return inputs.wheelAcceleration * SwerveConstants.Kinematics.wheelCircumference;
    }



    public double getVoltage() {
        return inputs.appliedDriveVoltage;
    }
}
