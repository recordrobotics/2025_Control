package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.SimpleMath;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {

  private static final String name = "limelight";
  private int numTags = 0;
  private double confidence = 0;
  private boolean hasVision = false;
  private boolean limelightConnected = false;
  private PoseEstimate currentEstimate = new PoseEstimate();
  private double currentConfidence = 9999999; // large number means less confident

  public Limelight() {
    LimelightHelpers.setPipelineIndex(name, 0);
    ShuffleboardUI.Overview.setTagNum(() -> numTags);
    ShuffleboardUI.Overview.setConfidence(() -> confidence);
    ShuffleboardUI.Overview.setHasVision(() -> hasVision);
    ShuffleboardUI.Overview.setLimelightConnected(() -> limelightConnected);
  }

  @Override
  public void periodic() {
    confidence = 0;
    LimelightHelpers.SetRobotOrientation(
        name, PoseTracker.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
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

    double timeSinceAuto = Timer.getFPGATimestamp() - Robot.getAutoStartTime();

    if (timeSinceAuto > 1
        && measurement
                .pose
                .getTranslation()
                .getDistance(PoseTracker.getEstimatedPosition().getTranslation())
            > 2) {
      confidence = 0;
    }

    handleMeasurement(measurement, confidence);
  }

  private void handleMeasurement(PoseEstimate estimate, double confidence) {
    if (confidence > 0) {
      hasVision = true;
      ShuffleboardUI.Autonomous.setVisionPose(estimate.pose);
      currentEstimate = estimate;
      currentConfidence = confidence;
    } else {
      hasVision = false;
      ShuffleboardUI.Autonomous.setVisionPose(new Pose2d());
      // TODO should it set confidence to 99999999999 because there is no vision?
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
    ShuffleboardUI.Overview.setTagNum(() -> numTags);
    ShuffleboardUI.Overview.setConfidence(() -> confidence);
    ShuffleboardUI.Overview.setHasVision(() -> hasVision);
    ShuffleboardUI.Overview.setLimelightConnected(() -> limelightConnected);
  }
}
