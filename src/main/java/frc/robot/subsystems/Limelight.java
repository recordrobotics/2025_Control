package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.libraries.LimelightHelpers;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {

  private static final String name = "limelight";
  private static final double SCORE_DISTANCE = 1;
  private int numTags = 0;
  private double confidence = 0;
  private boolean hasVision = false;
  private boolean limelightConnected = false;
  private PoseEstimate currentEstimate = new PoseEstimate();
  private double currentConfidence = 9999999; // large number means less confident

  public Limelight() {
    LimelightHelpers.setPipelineIndex(name, 0);
  }

  @Override
  public void periodic() {
    confidence = 0;
    LimelightHelpers.SetRobotOrientation(
        name,
        RobotContainer.poseTracker.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    PoseEstimate measurement_m2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (measurement == null || measurement_m2 == null) {
      limelightConnected = false;
      return;
    } else {
      limelightConnected = true;
    }

    numTags = measurement.tagCount;

    if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
      if (measurement.avgTagDist
          < Units.feetToMeters(7)) { // 7 feet is where the MT1 (yellow) gets bad wiggles
        confidence = 0.65; // mt 1
      } else {
        confidence = 0.7; // mt 2
        measurement = measurement_m2;
      }
    }

    if (measurement
            .pose
            .getTranslation()
            .getDistance(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
        > 2) {
      confidence = 0;
    }

    handleMeasurement(measurement, confidence);
    updateCrop();
  }

  private void updateCrop() {
    Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();
    double reefAngle =
        Math.atan2(
            pose.getTranslation().getY() - Constants.FieldPosition.ReefCenter.getPose().getY(),
            pose.getTranslation().getX() - Constants.FieldPosition.ReefCenter.getPose().getX());
    double processorAngle =
        Math.atan2(
            pose.getTranslation().getY() - Constants.FieldPosition.Processor.getPose().getY(),
            pose.getTranslation().getX() - Constants.FieldPosition.Processor.getPose().getX());

    if (pose.getTranslation().getDistance(Constants.FieldPosition.ReefCenter.getPose())
            < SCORE_DISTANCE
        || Math.abs(reefAngle - pose.getRotation().getRadians()) < Math.PI / 4) {
      setCrop(cropZones.REEF);
    } else if (pose.getTranslation().getDistance(Constants.FieldPosition.Processor.getPose())
            < SCORE_DISTANCE
        || Math.abs(processorAngle - pose.getRotation().getRadians()) < Math.PI / 4) {
      setCrop(cropZones.PROCESSOR);
    } else {
      setCrop(cropZones.Default);
    }
  }

  private void handleMeasurement(PoseEstimate estimate, double confidence) {
    if (confidence > 0) {
      hasVision = true;
      DashboardUI.Autonomous.setVisionPose(estimate.pose);
      currentEstimate = estimate;
      currentConfidence = confidence;
    } else {
      hasVision = false;
      DashboardUI.Autonomous.setVisionPose(new Pose2d());
      currentConfidence = 9999999;
    }
  }

  private void setCrop(cropZones zone) {
    LimelightHelpers.setCropWindow(name, zone.x1, zone.x2, zone.y1, zone.y2);
    LimelightHelpers.SetFiducialDownscalingOverride(name, zone.scale);
  }

  private enum cropZones {
    // x1, x2, y1, y2
    REEF(-1, 1, -1, 0, 1),
    PROCESSOR(-1, 1, 0, 1, 1),
    Default(-1, 1, -1, 1, 2);
    double x1, x2, y1, y2;
    float scale;

    cropZones(double x1, double x2, double y1, double y2, float scale) {
      this.x1 = x1;
      this.x2 = x2;
      this.y1 = y1;
      this.y2 = y2;
      this.scale = scale;
    }
  }

  public PoseEstimate getPoseEstimate() {
    return currentEstimate;
  }

  public double getConfidence() {
    return currentConfidence;
  }

  /** frees up all hardware allocations */
  public void close() {}

  @Override
  public void setupShuffleboard() {
    DashboardUI.Overview.setTagNum(() -> numTags);
    DashboardUI.Overview.setConfidence(() -> confidence);
    DashboardUI.Overview.setHasVision(() -> hasVision);
    DashboardUI.Overview.setLimelightConnected(() -> limelightConnected);
  }
}
