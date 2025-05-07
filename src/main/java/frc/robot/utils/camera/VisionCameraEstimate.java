package frc.robot.utils.camera;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import frc.robot.utils.libraries.LimelightHelpers.RawFiducial;

public class VisionCameraEstimate {

  public static class RawVisionFiducial {
    public int id = 0;
    public double area = 0;
    public double distToCamera = 0;
    public double distToRobot = 0;
    public double ambiguity = 0;

    public RawVisionFiducial(
        int id, double area, double distToCamera, double distToRobot, double ambiguity) {
      this.id = id;
      this.area = area;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
    }

    public RawVisionFiducial(RawFiducial limelightFiducial) {
      this(
          limelightFiducial.id,
          limelightFiducial.ta,
          limelightFiducial.distToCamera,
          limelightFiducial.distToRobot,
          limelightFiducial.ambiguity);
    }
  }

  public Pose2d pose;
  public double timestampSeconds;
  public double latency;
  public int tagCount;
  public double avgTagDist;
  public double avgTagArea;

  public RawVisionFiducial[] rawFiducials;
  public boolean isConstrained;

  public VisionCameraEstimate(
      Pose2d pose,
      double timestampSeconds,
      double latency,
      int tagCount,
      double avgTagDist,
      double avgTagArea,
      RawVisionFiducial[] rawFiducials,
      boolean isConstrained) {
    this.pose = pose;
    this.timestampSeconds = timestampSeconds;
    this.latency = latency;
    this.tagCount = tagCount;
    this.avgTagDist = avgTagDist;
    this.avgTagArea = avgTagArea;
    this.rawFiducials = rawFiducials;
    this.isConstrained = isConstrained;
  }

  public VisionCameraEstimate() {
    this(new Pose2d(), 0, 0, 0, 0, 0, new RawVisionFiducial[0], false);
  }

  private static RawVisionFiducial[] convertRawFiducials(RawFiducial[] rawFiducials) {
    RawVisionFiducial[] converted = new RawVisionFiducial[rawFiducials.length];
    for (int i = 0; i < rawFiducials.length; i++) {
      converted[i] = new RawVisionFiducial(rawFiducials[i]);
    }
    return converted;
  }

  public VisionCameraEstimate(PoseEstimate limelightEstimate) {
    this(
        limelightEstimate.pose,
        limelightEstimate.timestampSeconds,
        limelightEstimate.latency,
        limelightEstimate.tagCount,
        limelightEstimate.avgTagDist,
        limelightEstimate.avgTagArea,
        convertRawFiducials(limelightEstimate.rawFiducials),
        limelightEstimate.isMegaTag2);
  }
}
