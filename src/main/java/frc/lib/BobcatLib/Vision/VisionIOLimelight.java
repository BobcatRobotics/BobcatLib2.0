// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.BobcatLib.Vision;


import frc.lib.LimeLight.LimelightHelpersFast;

public class VisionIOLimelight implements VisionIO{
  /** Creates a new VisionIOLimelight. */
    LEDMode currentLedMode = LEDMode.FORCEOFF;
    CamMode currentCamMode = CamMode.VISION;
    public final limelightConstants constants;
    private final String name;

  public VisionIOLimelight(limelightConstants limelightConstants) {
    constants = limelightConstants;
    name = constants.name;
    
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ledMode = currentLedMode;
    inputs.camMode = currentCamMode;
    inputs.pipelineID = LimelightHelpersFast.getCurrentPipelineIndex(name);
    inputs.pipelineLatency = LimelightHelpersFast.getLatency_Pipeline(name);
    inputs.ta = LimelightHelpersFast.getTA(name);
    inputs.tv = LimelightHelpersFast.getTV(name);
    inputs.tx = LimelightHelpersFast.getTX(name);
    inputs.ty = LimelightHelpersFast.getTY(name);
    inputs.fiducialID = LimelightHelpersFast.getFiducialID(name);
    inputs.tClass=LimelightHelpersFast.getNeuralClassID(name);
    inputs.name=name;
  }



  @Override
  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case FORCEBLINK:
        LimelightHelpersFast.setLEDMode_ForceBlink(name);
        currentLedMode = LEDMode.FORCEBLINK;
        break;
      case FORCEOFF:
        LimelightHelpersFast.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
      case FORCEON:
        LimelightHelpersFast.setLEDMode_ForceOn(name);
        currentLedMode = LEDMode.FORCEON;
      case PIPELINECONTROL:
        LimelightHelpersFast.setLEDMode_PipelineControl(name);
        currentLedMode = LEDMode.PIPELINECONTROL;
      default:
        LimelightHelpersFast.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
        break;
    }
  }

  @Override
  public void setCamMode(CamMode mode){
    switch (mode){
      case DRIVERCAM:
      LimelightHelpersFast.setCameraMode_Driver(name);
      currentCamMode = CamMode.DRIVERCAM;
      case VISION:
      LimelightHelpersFast.setCameraMode_Processor(name);
      currentCamMode = CamMode.VISION;
    }
  }

  @Override
  public void setPipeline(String limelight, int index){    
    LimelightHelpersFast.setPipelineIndex(limelight, index);
  }
}
