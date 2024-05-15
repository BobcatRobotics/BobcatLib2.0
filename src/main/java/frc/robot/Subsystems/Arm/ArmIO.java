package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public Rotation2d angle;

    }
}
