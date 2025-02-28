package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import frc.robot.utils.libraries.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {
  @AutoLogOutput private int numTags = 0;

  private double confidence = 0;

  @AutoLogOutput private boolean hasVision = false;
  private String name = Constants.Limelight.LIMELIGHT_NAME;

  @AutoLogOutput private boolean limelightConnected = false;

  private double currentConfidence = 9999999; // large number means less confident
  private PoseEstimate currentEstimate = new PoseEstimate();
  private int cyclesSinceLastZoomOut = 0;
  private int cyclesDoneInZoomOut = 0;

  private double lastX0 = -1;
  private double lastX1 = 1;
  private double lastY0 = -1;
  private double lastY1 = 1;
  private int[] lastTagVisible = new int[22];
  private RawFiducial[] lastRawFiducials = new RawFiducial[22];
  private static final int savedFrames = 3;

  private PoseEstimate unsafeEstimate = new PoseEstimate();

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
      updateCrop(new RawFiducial[0]); // no fiducials
      return;
    } else {
      limelightConnected = true;
    }

    numTags = measurement.tagCount;

    if (!DashboardUI.Autonomous.getForceMT1()) {
      if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
        if (measurement.avgTagDist
            < Units.feetToMeters(7)) { // 7 feet is where the MT1 (yellow) gets bad wiggles
          confidence = 0.65; // mt 1
        } else {
          confidence = 0.7; // mt 2
          measurement = measurement_m2;
        }
      }
    } else {
      confidence = 0.65; // mt 1
    }

    unsafeEstimate = measurement;

    if (measurement
            .pose
            .getTranslation()
            .getDistance(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
        > 5) {
      confidence = 0;
    }

    handleMeasurement(measurement, confidence);
    updateCrop(measurement.rawFiducials);
  }

  // input (old, new)
  private void addMissingFiducials(RawFiducial[] rawFiducials1, RawFiducial[] rawFiducials2) {
    for (int i = 0; i < rawFiducials2.length; i++) {
      if (rawFiducials2[i] == null) {
        rawFiducials2[i] = rawFiducials1[i];
      }
    }
  }

  private void updateCrop(RawFiducial[] rawFiducials) {
    RawFiducial[] origRawFiducials = rawFiducials;

    addMissingFiducials(lastRawFiducials, rawFiducials);

    // clear lastRawFiducials and lastTagVisible
    for (int i = 0; i < lastRawFiducials.length; i++) {
      if (lastTagVisible[i] > 0) lastRawFiducials[i] = null;
      lastTagVisible[i]--;
    }

    // update lastRawFiducials and lastTagVisible
    for (RawFiducial f : origRawFiducials) {
      lastRawFiducials[f.id - 1] = f;
      lastTagVisible[f.id - 1] = savedFrames;
    }

    cyclesSinceLastZoomOut++;
    cyclesDoneInZoomOut++;

    if (cyclesDoneInZoomOut < 2) {
      LimelightHelpers.setCropWindow(name, -1, 1, -1, 1);
      LimelightHelpers.SetFiducialDownscalingOverride(name, 2);
      cyclesSinceLastZoomOut = 0;

      Logger.recordOutput("IsZoomedOut", true);
      return;
    }

    if (cyclesSinceLastZoomOut > Constants.Limelight.MAX_CYCLES_UNTIL_ZOOM_OUT
        || rawFiducials.length == 0) {
      LimelightHelpers.setCropWindow(name, -1, 1, -1, 1);
      LimelightHelpers.SetFiducialDownscalingOverride(name, 2);
      cyclesSinceLastZoomOut = 0;
      cyclesDoneInZoomOut = 0;

      Logger.recordOutput("IsZoomedOut", true);
      return;
    }

    Logger.recordOutput("IsZoomedOut", false);

    // values that will definatly be cropped in
    double x0 = rawFiducials[0].txnc / Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER.in(Degrees);
    double x1 = rawFiducials[0].txnc / Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER.in(Degrees);
    double y0 = rawFiducials[0].tync / Constants.Limelight.FOV_VERTICAL_FROM_CENTER.in(Degrees);
    double y1 = rawFiducials[0].tync / Constants.Limelight.FOV_VERTICAL_FROM_CENTER.in(Degrees);

    for (RawFiducial f : rawFiducials) {
      double tagX0 = f.txnc / Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER.in(Degrees);
      double tagX1 = f.txnc / Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER.in(Degrees);
      double tagY0 = f.tync / Constants.Limelight.FOV_VERTICAL_FROM_CENTER.in(Degrees);
      double tagY1 = f.tync / Constants.Limelight.FOV_VERTICAL_FROM_CENTER.in(Degrees);

      Double angleFromCenterToEdge =
          Math.atan(
              (Constants.Limelight.DISTANCE_FROM_TAG_CENTER_TO_EDGE
                      + Constants.Limelight.DISTANCE_CROPPING_MARGIN)
                  / f.distToCamera);

      double verticalMargin =
          Constants.Limelight.ANGLE_CROPPING_MARGIN
              + (angleFromCenterToEdge / Constants.Limelight.FOV_VERTICAL_FROM_CENTER.in(Radians));
      double horizontalMargin =
          Constants.Limelight.ANGLE_CROPPING_MARGIN
              + (angleFromCenterToEdge
                  / Constants.Limelight.FOV_HORIZONTAL_FROM_CENTER.in(Radians));

      tagX0 -= horizontalMargin;
      tagX1 += horizontalMargin;
      tagY0 -= verticalMargin;
      tagY1 += verticalMargin;

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

    // ensure values are in [-1, 1]
    if (x0 < -1) {
      x0 = -1;
    }
    if (x1 > 1) {
      x1 = 1;
    }
    if (y0 < -1) {
      y0 = -1;
    }
    if (y1 > 1) {
      y1 = 1;
    }

    lastX0 = x0;
    lastY0 = y0;
    lastX1 = x1;
    lastY1 = y1;

    // average of all x0, x1, y0, y1 values with their prevs
    x0 = (x0 + lastX0) / 2;
    x1 = (x1 + lastX1) / 2;
    y0 = (y0 + lastY0) / 2;
    y1 = (y1 + lastY1) / 2;

    LimelightHelpers.setCropWindow(name, x0, x1, y0, y1);
    LimelightHelpers.SetFiducialDownscalingOverride(name, 1);
  }

  private void handleMeasurement(PoseEstimate estimate, double confidence) {
    if (confidence > 0) {
      hasVision = true;
      DashboardUI.Autonomous.setVisionPose(estimate.pose);
      currentEstimate = estimate;
      currentConfidence = confidence;
      Logger.recordOutput("Limelight/Pose", estimate.pose);
    } else {
      hasVision = false;
      DashboardUI.Autonomous.setVisionPose(estimate.pose);
      currentConfidence = 9999999;
      Logger.recordOutput("Limelight/Pose", estimate.pose);
    }
  }

  public PoseEstimate getPoseEstimate() {
    return currentEstimate;
  }

  public PoseEstimate getUnsafePoseEstimate() {
    return unsafeEstimate;
  }

  @AutoLogOutput
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
