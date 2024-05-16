package frc.robot.Subsystems.Superstructure.Arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public Rotation2d angle;
        public Rotation2d desiredAngle;
    }

    public default void updateInputs(ArmIOInputs inputs){}

    public default void setAngle(Rotation2d angle){}

}
