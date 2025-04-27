package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {
  public enum LimelightType {
    Limelight2(1280, 960, 2, 74.36, 33, 0.007, 0.0005, 35, 5),
    Limelight3G(
        1280, 960, 1, 100, 38, 0.007, 0.0005, 35,
        5); // TODO: what is the actual fov of new lens we replaced

    int width;
    int height;
    double downscale;
    double fov;
    double fps;
    double pxError;
    double pxErrorStdDev;
    double latencyMs;
    double latencyStdDevMs;

    LimelightType(
        int width,
        int height,
        double downscale,
        double fov,
        double fps,
        double pxError,
        double pxErrorStdDev,
        double latencyMs,
        double latencyStdDevMs) {
      this.width = width;
      this.height = height;
      this.downscale = downscale;
      this.fov = fov;
      this.fps = fps;
      this.pxError = pxError;
      this.pxErrorStdDev = pxErrorStdDev;
      this.latencyMs = latencyMs;
      this.latencyStdDevMs = latencyStdDevMs;
    }

    public int getDetectorWidth() {
      return (int) Math.floor(width / downscale);
    }

    public int getDetectorHeight() {
      return (int) Math.floor(height / downscale);
    }
  }

  public static class LimelightData {
    public int numTags = 0;
    private double confidence = 0;
    public boolean hasVision = false;
    public boolean limelightConnected = false;

    public double currentConfidence = 9999999; // large number means less confident
    public PoseEstimate currentEstimate = new PoseEstimate();
    public PoseEstimate unsafeEstimate = new PoseEstimate();

    public String name;

    private final double MT1_CONFIDENCE;
    private final double MT2_CONFIDENCE;

    private final double MT1_MAX_DIST;

    private final double MAX_POSE_ERROR;

    private PhotonCamera fakeCamera;
    private PhotonPoseEstimator photonEstimator;
    private LimelightType type;

    public LimelightData(String name, LimelightType type, Transform3d robotToCamera) {
      this.name = name;
      this.type = type;
      this.MT1_CONFIDENCE = 0.65;
      this.MT2_CONFIDENCE = 0.7;
      this.MT1_MAX_DIST =
          Units.feetToMeters(7); // 7 feet is where the MT1 (yellow) gets bad wiggles
      this.MAX_POSE_ERROR = 5; // 5 meters

      if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
        fakeCamera = new PhotonCamera(name);
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
            type.getDetectorWidth(), type.getDetectorHeight(), Rotation2d.fromDegrees(type.fov));
        cameraProp.setCalibError(type.pxError, type.pxErrorStdDev);
        cameraProp.setFPS(type.fps);
        cameraProp.setAvgLatencyMs(type.latencyMs);
        cameraProp.setLatencyStdDevMs(type.latencyStdDevMs);

        PhotonCameraSim cameraSim = new PhotonCameraSim(fakeCamera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        RobotContainer.visionSim.addCamera(cameraSim, robotToCamera);

        photonEstimator =
            new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
      }
    }

    public LimelightData(
        String name,
        double mt1_confidence,
        double mt2_confidence,
        double mt1_max_dist,
        double max_pose_error) {
      this.name = name;
      this.MT1_CONFIDENCE = mt1_confidence;
      this.MT2_CONFIDENCE = mt2_confidence;
      this.MT1_MAX_DIST = mt1_max_dist;
      this.MAX_POSE_ERROR = max_pose_error;
    }

    public void setPipeline(int pipeline) {
      LimelightHelpers.setPipelineIndex(name, pipeline);
    }

    public void updateEstimation() {
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

      if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        var results = fakeCamera.getAllUnreadResults();
        if (results.size() > 0) {
          var change = results.get(results.size() - 1);
          visionEst = photonEstimator.update(change);

          if (visionEst.isPresent()) {
            var est = visionEst.get();
            int numTags = est.targetsUsed.size();

            double avgTagDist = 0;
            double avgTagArea = 0;
            double minx = 1;
            double miny = 1;
            double maxx = -1;
            double maxy = -1;

            List<RawFiducial> rawFiducials = new ArrayList<>(numTags);
            for (var target : est.targetsUsed) {
              double dist = target.getBestCameraToTarget().getTranslation().getNorm();
              double distRobot =
                  target
                      .getBestCameraToTarget()
                      .plus(photonEstimator.getRobotToCameraTransform())
                      .getTranslation()
                      .getNorm();
              avgTagDist += dist;
              avgTagArea += target.getArea();

              double tcx = 0;
              double tcy = 0;
              for (var corner : target.getDetectedCorners()) {
                tcx += corner.x;
                tcy += corner.y;

                if (corner.x < minx) {
                  minx = corner.x;
                }
                if (corner.x > maxx) {
                  maxx = corner.x;
                }
                if (corner.y < miny) {
                  miny = corner.y;
                }
                if (corner.y > maxy) {
                  maxy = corner.y;
                }
              }
              tcx /= target.getDetectedCorners().size();
              tcy /= target.getDetectedCorners().size();

              var rawFiducial =
                  new RawFiducial(
                      target.fiducialId,
                      tcx,
                      tcy,
                      target.getArea(),
                      dist,
                      distRobot,
                      target.getPoseAmbiguity());
              rawFiducials.add(rawFiducial);
            }
            avgTagDist /= numTags;
            avgTagArea /= numTags;

            double span = Math.sqrt((maxx - minx) * (maxy - miny));

            measurement =
                new PoseEstimate(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    type.latencyMs,
                    numTags,
                    span,
                    avgTagDist,
                    avgTagArea,
                    rawFiducials.toArray(new RawFiducial[0]),
                    false);
            measurement_m2 =
                new PoseEstimate(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    type.latencyMs,
                    numTags,
                    span,
                    avgTagDist,
                    avgTagArea,
                    rawFiducials.toArray(new RawFiducial[0]),
                    true);
          }
        }
      }

      if (measurement == null || measurement_m2 == null) {
        limelightConnected = false;
        return;
      } else {
        limelightConnected = true;
      }

      numTags = measurement.tagCount;

      if (!DashboardUI.Autonomous.getForceMT1()) {
        if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
          if (measurement.avgTagDist < MT1_MAX_DIST) {
            confidence = MT1_CONFIDENCE;
          } else {
            confidence = MT2_CONFIDENCE;
            measurement = measurement_m2;
          }
        }
      } else {
        confidence = MT1_CONFIDENCE;
      }

      unsafeEstimate = measurement;

      if (measurement
              .pose
              .getTranslation()
              .getDistance(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
          > MAX_POSE_ERROR) {
        confidence = 0;
      }

      if (confidence > 0) {
        hasVision = true;
        currentEstimate = measurement;
        currentConfidence = confidence;
      } else {
        hasVision = false;
        currentConfidence = 9999999;
      }
    }

    public void logValues(String id) {
      String prefix = "Limelight/" + id + "/";
      Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose);
      Logger.recordOutput(prefix + "NumTags", numTags);
      Logger.recordOutput(prefix + "Confidence", currentConfidence);
      Logger.recordOutput(prefix + "HasVision", hasVision);
      Logger.recordOutput(prefix + "LimelightConnected", limelightConnected);
    }
  }

  private LimelightData left =
      new LimelightData(
          Constants.Limelight.LIMELIGHT_LEFT_NAME,
          LimelightType.Limelight3G,
          Constants.Limelight.leftTransformRobotToCamera);
  private LimelightData center =
      new LimelightData(
          Constants.Limelight.LIMELIGHT_CENTER_NAME,
          LimelightType.Limelight2,
          Constants.Limelight.centerTransformRobotToCamera);

  public Limelight() {
    left.setPipeline(0);
    center.setPipeline(0);
  }

  @Override
  public void periodic() {
    left.updateEstimation();
    center.updateEstimation();

    DashboardUI.Autonomous.setVisionPoseLeft(left.unsafeEstimate.pose);
    DashboardUI.Autonomous.setVisionPoseCenter(center.unsafeEstimate.pose);

    left.logValues("Left");
    center.logValues("Center");
  }

  public LimelightData getLeft() {
    return left;
  }

  public LimelightData getCenter() {
    return center;
  }

  /** frees up all hardware allocations */
  public void close() {}

  @Override
  public void setupShuffleboard() {
    DashboardUI.Overview.setTagNumLeft(() -> left.numTags);
    DashboardUI.Overview.setConfidenceLeft(() -> left.confidence);
    DashboardUI.Overview.setHasVisionLeft(() -> left.hasVision);
    DashboardUI.Overview.setLimelightConnectedLeft(() -> left.limelightConnected);

    DashboardUI.Overview.setTagNumCenter(() -> center.numTags);
    DashboardUI.Overview.setConfidenceCenter(() -> center.confidence);
    DashboardUI.Overview.setHasVisionCenter(() -> center.hasVision);
    DashboardUI.Overview.setLimelightConnectedCenter(() -> center.limelightConnected);
  }
}
