package BobcatLib.Team177.Vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class limelightConstants {
  /** Limelight Name */
  public final String name;
  /** Vertical FOV */
  public final double verticalFOV;
  /** Horizontal FOV */
  public final double horizontalFOV;
  /** Limelight Mount Height */
  public final double limelightMountHeight;
  /** Detector Pipeline Index */
  public final int detectorPiplineIndex;
  /** AprilTag Pipeline Index */
  public final int apriltagPipelineIndex;
  /** Horizontal Pixels */
  public final int horPixels;
  /** Vision Standard Devs */
  public final Vector<N3> visionMeasurementStdDevs;

  /**
   * Limelight Constants to be used when creating swerve modules.
   *
   * @param name name
   * @param verticalFOV VFOV
   * @param horizontalFOV HFOV
   * @param limelightMountHeight Mount Height
   * @param detectorPiplineIndex Detector Pipeline Index
   * @param apriltagPipelineIndex AprilTag Pipeline Index
   * @param horPixels Horizontal Pixels
   * @param visionMeasurementStdDevs Vision Measurement Std Deviations
   */
  public limelightConstants(
      String name,
      double verticalFOV,
      double horizontalFOV,
      double limelightMountHeight,
      int detectorPiplineIndex,
      int apriltagPipelineIndex,
      int horPixels,
      Vector<N3> visionMeasurementStdDevs) {
    this.name = name;
    this.verticalFOV = verticalFOV; // degrees obviously
    this.horizontalFOV = horizontalFOV;
    this.limelightMountHeight = limelightMountHeight;
    this.detectorPiplineIndex = detectorPiplineIndex;
    this.apriltagPipelineIndex = apriltagPipelineIndex;
    this.horPixels = horPixels;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
  }
}
