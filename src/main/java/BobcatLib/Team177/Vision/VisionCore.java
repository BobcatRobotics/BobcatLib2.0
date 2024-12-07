package BobcatLib.Team177.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionCore {

    /**
     * for Limelight, we usually use getBotPoseEstimate_wpiBlue_MegaTag2() to get this value
     * @return the estimated pose of the robot
     */
    public default Pose2d getBotPose(){
        return new Pose2d();
    }

    public default String getName(){
        return "Camera not properly initialized!";
    }

    public default double getTagCount(){
        return -1;
    }


    /**
     * is the pose of the robot possible and reasonable?
     */
    public default boolean getPoseValid(Rotation2d gyro){
        return false;
    }

    /**
     * what time was the pose taken? this helps us account for latency
     * 
     * for limelight we use LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).timestampSeconds
     * 
     */
    public default double getPoseTimestamp(){
        return -1;
    }

    /**
     * Tells the robot what direction it is facing, 
     * necessary for the megatag2 algorithm
     */
    public default void setRobotOrientation(Rotation2d gyro){}


    /**
     * Which vision tags to whitelist
     */
    public default void setPermittedTags(int[] tags){}

}