package frc.robot.StateMachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState{
    public Pose2d EEPos;
    public Pose2d armPos;
    public Pose2d wristPos;
    public Pose2d elevatorPos; //meters

    public RobotState(Pose2d EEPos){
        this.EEPos = EEPos;
    }
    /**
     * 
     * @param elevatorDistMeters distance of the elevator along its axis, not just x or y
     * @param arm global rotation of arm
     * @param wrist global rotation of wrist
     */
    public RobotState(double elevatorDistMeters, Rotation2d arm, Rotation2d wrist){

    }
}