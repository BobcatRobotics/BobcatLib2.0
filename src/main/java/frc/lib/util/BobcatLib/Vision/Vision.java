// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.BobcatLib.Vision;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.BobcatLib.Team254.BobcatUtil;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public boolean apriltagPipeline;

  public Vision(VisionIO io) {
    this.io = io;

    io.setLEDS(LEDMode.FORCEOFF);
  }

  public void setLEDS(boolean on) {
    io.setLEDS(on ? LEDMode.FORCEBLINK : LEDMode.PIPELINECONTROL);
  }

  public void setCamMode(CamMode mode) {
    io.setCamMode(mode);
  }

  public double getTClass() {
    return inputs.tClass;
  }

  public boolean getTV() {
    return inputs.tv;
  }

  public double getID(){
    return inputs.fiducialID;
  }

  public void setPipeline(int id) {
    io.setPipeline(inputs.name, id);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Limelight" + inputs.name, inputs);

    apriltagPipeline = inputs.pipelineID == 0;
  }



  public Pose2d getBotPoseMG2(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(inputs.name).pose;
  }

  public Pose2d getPoseMG2(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(inputs.name).pose;
  }

  public LimelightHelpers.PoseEstimate getPoseEstimateMG2(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(inputs.name);
  }

  /**
   * tells the limelight what the rotation of the gyro is, for determining pose ambiguity stuff
   */
  public void SetRobotOrientation(Rotation2d gyro){
    gyro = BobcatUtil.isBlue()? gyro : gyro.rotateBy(Rotation2d.fromDegrees(180));
    double gyroval = Math.toDegrees(BobcatUtil.get0to2Pi(gyro.getRadians()));
    
    LimelightHelpers.SetRobotOrientation(inputs.name, gyroval, 0, 0, 0, 0, 0);
  }

  /**
   * 
   * @param tags anything NOT in here will be thrownOut
   */
  public void setPermittedTags(int[] tags){
      LimelightHelpers.SetFiducialIDFiltersOverride(inputs.name, tags);
  }

  /**
   * 
   * 
   * 
   */
  public boolean getPoseValidMG2(Rotation2d gyro){
    
    //get raw data from limelight pose estimator
    Pose2d botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(inputs.name).pose;
    double diff = 0;

    double gyroval=gyro.getDegrees();
    gyroval = gyroval % (360);

    double x = botpose.getX();
    double y = botpose.getY();

    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(inputs.name);
    double tagDist = poseEstimate.avgTagDist;

    //debugging purposes only
    Logger.recordOutput("LLDebug/"+inputs.name+" avgTagDist", tagDist);
    Logger.recordOutput("LLDebug/"+inputs.name+" tagCount", poseEstimate.tagCount);
    Logger.recordOutput("LLDebug/"+inputs.name+" x val", x);
    Logger.recordOutput("LLDebug/"+inputs.name+" y val", y);
    Logger.recordOutput("LLDebug/"+inputs.name+" rdiff", diff);


    // this determines if the raw data from the limelight is valid
    // sometimes the limelight will give really bad data, so we want to throw this out
    // and not use it in our pose estimation.
    // to check for this, we check to see if the rotation from the pose matches
    // the rotation that the gyro is reporting
    // we then check if the pose is actually within the bounds of the field
    // if all these requirements are met, then we can trust the measurement
    // otherwise we ignore it.

    if( 
        (diff<VisionConstants.rotationTolerance) && 
        (tagDist<VisionConstants.throwoutDist) &&
        (botpose.getTranslation().getX() > 0) &&
        (botpose.getTranslation().getX() < FieldConstants.fieldLength) &&
        (botpose.getTranslation().getY() > 0) &&
        (botpose.getTranslation().getY() < FieldConstants.fieldWidth)) {
          
          return true;
      } else{
          return false;
      }


  }

  public Pose3d getBotPose3d() {
    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(inputs.name);
    Logger.recordOutput("Limelight" + inputs.name + "/Pose3d", pose);
    return pose;

  }

  // public double getDistToTag() {
  //   //indicies don't match documentation with targetpose_robotspace
  //   Logger.recordOutput("Limelight" + inputs.name + "/distanceToTagHypot", Math.hypot(LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0], LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]));
  //   return Math.hypot(LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0], LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]); // 0 is x, 2 is z 
    
  // }


  public double getPoseTimestampMG2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(inputs.name).timestampSeconds;
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(inputs.name);
  }


   public String getLimelightName(){
    return inputs.name;
   }

  // angle target is from the center of the limelights crosshair
  public Rotation2d getTX() {
    return Rotation2d.fromDegrees(inputs.tx);
  }

  public double getTA() {
    return inputs.ta;
  }

  public void setPriorityID(int tagID, String limelightID) {
    NetworkTableInstance.getDefault().getTable(limelightID).getEntry("priorityid").setDouble(tagID);
  }

}
