package frc.lib.BobcatLib.Swerve.SwerveModule;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.BobcatLib.Swerve.SwerveConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private FlywheelSim driveSim;
    private FlywheelSim angleSim;
    private double loopPeriodSecs;
    private double angleAbsolutePosRot = Math.random();
    public SwerveModuleIOSim(double loopPeriodSecs) {
        // Using flywheels to simulate motors
        driveSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.driveGearRatio, 0.025);
        angleSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.angleGearRatio, 0.004);
        this.loopPeriodSecs = loopPeriodSecs;
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(loopPeriodSecs);
        angleSim.update(loopPeriodSecs);

        double angleDiffRot = (angleSim.getAngularVelocityRPM() / 60) * loopPeriodSecs;
        angleAbsolutePosRot += angleDiffRot;
        while (angleAbsolutePosRot < 0) {
            angleAbsolutePosRot += 1;
        }
        while (angleAbsolutePosRot > 1) {
            angleAbsolutePosRot -= 1;
        }

        inputs.wheelPositionRot += (driveSim.getAngularVelocityRPM() / 60) * loopPeriodSecs;
        inputs.wheelVelocityRotPerSec = driveSim.getAngularVelocityRPM() / 60;
        inputs.canCoderPositionRot = angleAbsolutePosRot;
    }

    /**
     * Sets the percent out of the drive motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setDrivePercentOut(double percent) {
        double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
        driveSim.setInputVoltage(volts);
    }

    /**
     * Stops the drive motor
     */
    public void stopDrive() {
        driveSim.setInputVoltage(0);
    }

    /**
     * Sets the percent out of the angle motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setAnglePercentOut(double percent) {
        double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
        angleSim.setInputVoltage(volts);
    }

    /**
     * Stops the angle motor
     */
    public void stopAngle() {
        angleSim.setInputVoltage(0);
    }
}
