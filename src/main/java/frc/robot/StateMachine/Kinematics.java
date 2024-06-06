package frc.robot.StateMachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Kinematics {
    public Pose2d arm;
    public final double armLength = 0.25;
    public Pose2d wrist;
    public final double wristLength = 0.5;
    public Pose2d elevator;
    public final Rotation2d elevatorAngle = Rotation2d.fromDegrees(45);


    public Kinematics(){}

    
}
