// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Subsystems.apriltagvision;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.apriltagvision.AprilTagVisionFieldConstants.AprilTagLayoutType;
import lombok.Getter;

public class AprilTagVisionConstants {
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double fieldWidth = Units.inchesToMeters(323.277);

  public static final double ambiguityThreshold;
  public static final double targetLogTimeSecs;
  public static final double fieldBorderMargin;
  public static final double zMargin;
  public static final double xyStdDevCoefficient;
  public static final double thetaStdDevCoefficient;

  public static final double[] stdDevFactors;

  public static final Pose3d[] cameraPoses;

  public static final String[] instanceNames;

  public static final String[] cameraIds;
  private int[] cameraResolutionWidth;
  private static final int[] cameraResolutionHeight;
  private static final int cameraAutoExposure;
  // private static final int cameraExposure = 10;
  private static final int cameraGain[];
  private static final int maxFPS[];
  private int cameraExposure[];
  // private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  
  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2024-official"),
    // SPEAKERS_ONLY("2024-speakers"),
    // AMPS_ONLY("2024-amps"),
    // WPI("2024-wpi"),
    CUSTOM("custom"),
    CUSTOM2("custom2");

    private AprilTagLayoutType(String name) {
        try {
          layout =
              new AprilTagFieldLayout(
                  Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      
      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException(
              "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }

  public AprilTagVisionConstants(int[] cameraResolutionWidth){
    this.cameraResolutionWidth = cameraResolutionWidth;

  }

}
