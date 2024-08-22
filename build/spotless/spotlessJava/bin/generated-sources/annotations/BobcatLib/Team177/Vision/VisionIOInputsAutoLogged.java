package BobcatLib.Team177.Vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LedMode", ledMode);
    table.put("PipelineID", pipelineID);
    table.put("PipelineLatency", pipelineLatency);
    table.put("Ta", ta);
    table.put("Tv", tv);
    table.put("Tx", tx);
    table.put("Ty", ty);
    table.put("FiducialID", fiducialID);
    table.put("TClass", tClass);
    table.put("Name", name);
    table.put("CamMode", camMode);
    table.put("BotPoseMG2", botPoseMG2);
    table.put("TagCount", tagCount);
    table.put("AvgTagDist", avgTagDist);
    table.put("BotPose3d", botPose3d);
    table.put("Timestamp", timestamp);
  }

  @Override
  public void fromLog(LogTable table) {
    ledMode = table.get("LedMode", ledMode);
    pipelineID = table.get("PipelineID", pipelineID);
    pipelineLatency = table.get("PipelineLatency", pipelineLatency);
    ta = table.get("Ta", ta);
    tv = table.get("Tv", tv);
    tx = table.get("Tx", tx);
    ty = table.get("Ty", ty);
    fiducialID = table.get("FiducialID", fiducialID);
    tClass = table.get("TClass", tClass);
    name = table.get("Name", name);
    camMode = table.get("CamMode", camMode);
    botPoseMG2 = table.get("BotPoseMG2", botPoseMG2);
    tagCount = table.get("TagCount", tagCount);
    avgTagDist = table.get("AvgTagDist", avgTagDist);
    botPose3d = table.get("BotPose3d", botPose3d);
    timestamp = table.get("Timestamp", timestamp);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.ledMode = this.ledMode;
    copy.pipelineID = this.pipelineID;
    copy.pipelineLatency = this.pipelineLatency;
    copy.ta = this.ta;
    copy.tv = this.tv;
    copy.tx = this.tx;
    copy.ty = this.ty;
    copy.fiducialID = this.fiducialID;
    copy.tClass = this.tClass;
    copy.name = this.name;
    copy.camMode = this.camMode;
    copy.botPoseMG2 = this.botPoseMG2;
    copy.tagCount = this.tagCount;
    copy.avgTagDist = this.avgTagDist;
    copy.botPose3d = this.botPose3d;
    copy.timestamp = this.timestamp;
    return copy;
  }
}
