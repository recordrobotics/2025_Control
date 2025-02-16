package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.libraries.LimelightHelpers;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {
  private int numTags = 0;
  private double confidence = 0;
  private boolean hasVision = false;
  private String name = Constants.Limelight.LIMELIGHT_NAME;
  private boolean limelightConnected = false;
  private double currentConfidence = 9999999; // large number means less confident
  private PoseEstimate currentEstimate = new PoseEstimate();

  public Limelight() {
    LimelightHelpers.setPipelineIndex(name, 0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "limelightPose",
        new Pose3d(
            new Translation3d(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
                .plus(Constants.Limelight.LIMELIGHT_OFFSET),
            new Rotation3d(
                Degrees.of(0),
                Constants.Limelight.LIMELIGHT_ANGLE_UP,
                RobotContainer.poseTracker.getEstimatedPosition().getRotation().getMeasure())));

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
      updateCrop(); // TODO if the limelights not connected, why update the crop
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

  private List<Pose3d> visibleTags = new ArrayList<>();

  private void updateCrop() {
    if (!hasVision
        && (Constants.RobotState.getMode()
            == Mode.REAL)) { // failsafe. if crop wrong this trigger; reset crop
      LimelightHelpers.setCropWindow(name, -1, 1, -1, 1);
      LimelightHelpers.SetFiducialDownscalingOverride(name, 2);
      return;
    }

    Pose2d robotPose = RobotContainer.poseTracker.getEstimatedPosition();
    Pose3d limelightPose3d =
        new Pose3d(
            new Translation3d(robotPose.getTranslation())
                .plus(Constants.Limelight.LIMELIGHT_OFFSET)
                .rotateAround(
                    new Translation3d(robotPose.getTranslation()),
                    new Rotation3d(0, 0, robotPose.getRotation().getRadians())),
            new Rotation3d(
                Degrees.of(0),
                Constants.Limelight.LIMELIGHT_ANGLE_UP.unaryMinus(),
                robotPose.getRotation().getMeasure()));

    boolean foundOne = false;

    double x0 = 1;
    double y0 = 1;
    double x1 = -1;
    double y1 = -1;

    visibleTags.clear();

    for (AprilTag tag : Constants.Limelight.FIELD_LAYOUT.getTags()) {
      if (tag.ID != 13) {
        continue;
      }

      Translation3d limelightToTagTranslation =
          tag.pose.getTranslation().minus(limelightPose3d.getTranslation());
      Rotation3d limelightToTagRotation =
          SimpleMath.translationToRotation(limelightToTagTranslation);

      // using isNear feels cursed but if it works it works
      boolean withinRangeVertical =
          limelightToTagRotation
              .getMeasureY()
              .isNear(
                  Constants.Limelight.LIMELIGHT_ANGLE_UP.unaryMinus(),
                  Constants.Limelight.FOV_VERTICAL_FROM_CENTER);
      boolean withinRangeHorizontal =
          limelightToTagRotation
              .getMeasureZ()
              .isNear(
                  limelightPose3d.getRotation().getMeasureZ(),
                  Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER);
      boolean isCloseEnough =
          limelightToTagTranslation.getNorm() < Constants.Limelight.MAX_SIGHT_DISTANCE;

      if (withinRangeVertical && withinRangeHorizontal && isCloseEnough) {
        foundOne = true;

        visibleTags.add(new Pose3d(tag.pose.getTranslation(), limelightToTagRotation));

        double tagCenterX =
            -((limelightToTagRotation.getMeasureZ().in(Degrees)
                    - limelightPose3d.getRotation().getMeasureZ().in(Degrees))
                / Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER.in(Degrees));
        double tagCenterY =
            (limelightToTagRotation.getMeasureY().in(Degrees)
                    - limelightPose3d.getRotation().getMeasureY().in(Degrees))
                / Constants.Limelight.FOV_VERTICAL_FROM_CENTER.in(Degrees);
        double tagX0 = tagCenterX - Constants.Limelight.CROPPING_MARGIN;
        double tagX1 = tagX0 + 2 * Constants.Limelight.CROPPING_MARGIN;
        double tagY0 = tagCenterY - Constants.Limelight.CROPPING_MARGIN;
        double tagY1 = tagY0 + 2 * Constants.Limelight.CROPPING_MARGIN;

        if (tagX0 < x0) {
          x0 = tagX0;
        }
        if (tagX1 > x1) {
          x1 = tagX1;
        }
        if (tagY0 < y0) {
          y0 = tagY0;
        }
        if (tagY1 > y1) {
          y1 = tagY1;
        }
      }
    }

    // Make sure x0, x1, y0, y1 are within the range of -1 to 1
    x0 = Math.max(-1, x0);
    x1 = Math.min(1, x1);
    y0 = Math.max(-1, y0);
    y1 = Math.min(1, y1);

    if (!foundOne) {
      LimelightHelpers.setCropWindow(name, -1, 1, -1, 1);
      LimelightHelpers.SetFiducialDownscalingOverride(name, 2);
      Logger.recordOutput("x0", -1.0);
      Logger.recordOutput("x1", 1.0);
      Logger.recordOutput("y0", -1.0);
      Logger.recordOutput("y1", 1.0);
      Logger.recordOutput("downscale", 2.0);
      return;
    }
    LimelightHelpers.setCropWindow(name, x0, x1, y0, y1);
    LimelightHelpers.SetFiducialDownscalingOverride(name, 1);
    Logger.recordOutput("x0", x0);
    Logger.recordOutput("x1", x1);
    Logger.recordOutput("y0", y0);
    Logger.recordOutput("y1", y1);
    Logger.recordOutput("downscale", 1.0);
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

  @AutoLogOutput
  private Pose3d[] getVisibleTags() {
    Pose3d[] poses = new Pose3d[visibleTags.size()];
    for (int i = 0; i < visibleTags.size(); i++) {
      poses[i] = visibleTags.get(i);
    }
    return poses;
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
