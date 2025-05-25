package frc.robot.utils.camera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.VisionSimulationMode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
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

public class LimelightCamera implements IVisionCamera {

  private int numTags = 0;
  private double confidence = 0;
  private boolean hasVision = false;
  private boolean limelightConnected = false;

  private double currentConfidence = 9999999; // large number means less confident
  private VisionCameraEstimate currentEstimate = new VisionCameraEstimate();
  private VisionCameraEstimate unsafeEstimate = new VisionCameraEstimate();

  private String name;

  private final double MT1_CONFIDENCE;
  private final double MT2_CONFIDENCE;

  private final double MT1_MAX_DIST;

  private final double MAX_POSE_ERROR;

  private PhotonCamera fakeCamera;
  private PhotonPoseEstimator photonEstimator;
  private CameraType type;

  private static final RawFiducial[] ALL_SIM_TAGS =
      Constants.Game.APRILTAG_LAYOUT.getTags().stream()
          .map(v -> new RawFiducial(v.ID, 0.1, 0.1, 0.1, 6, 7, 0))
          .toArray(RawFiducial[]::new);

  public boolean isConnected() {
    return limelightConnected;
  }

  public String getName() {
    return name;
  }

  public CameraType getCameraType() {
    return type;
  }

  public boolean hasVision() {
    return hasVision;
  }

  public int getNumTags() {
    return numTags;
  }

  public VisionCameraEstimate getCurrentEstimate() {
    return currentEstimate;
  }

  public VisionCameraEstimate getUnsafeEstimate() {
    return unsafeEstimate;
  }

  public double getConfidence() {
    return currentConfidence;
  }

  public double getUnprocessedConfidence() {
    return confidence;
  }

  public LimelightCamera(String name, CameraType type, Transform3d robotToCamera) {
    this.name = name;
    this.type = type;
    this.MT1_CONFIDENCE = 0.65;
    this.MT2_CONFIDENCE = 0.7;
    this.MT1_MAX_DIST = Units.feetToMeters(7); // 7 feet is where the MT1 (yellow) gets bad wiggles
    this.MAX_POSE_ERROR = 5; // 5 meters

    if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL
        && Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM) {
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
              Constants.Game.APRILTAG_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              robotToCamera);
    }
  }

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  public void updateEstimation(boolean trust) {
    confidence = 0;
    LimelightHelpers.SetRobotOrientation(
        name,
        RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    PoseEstimate measurement_m2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL
        && Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM) {
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
    } else if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
      var maplePose = RobotContainer.model.getRobot();
      if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.MAPLE_NOISE) {
        maplePose = SimpleMath.poseNoise(maplePose, 0.001, 0.001);
      }

      measurement =
          new PoseEstimate(
              maplePose,
              Timer.getFPGATimestamp(),
              type.latencyMs,
              ALL_SIM_TAGS.length,
              0.1,
              6.0,
              2.0,
              ALL_SIM_TAGS,
              false);
      measurement_m2 =
          new PoseEstimate(
              maplePose,
              Timer.getFPGATimestamp(),
              type.latencyMs,
              ALL_SIM_TAGS.length,
              0.1,
              6.0,
              2.0,
              ALL_SIM_TAGS,
              true);
    }

    if (measurement == null || measurement_m2 == null) {
      if (Constants.RobotState.getMode() != Constants.RobotState.Mode.SIM) {
        limelightConnected = false; // only can be disconnected out of sim
      }
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

    unsafeEstimate = new VisionCameraEstimate(measurement);

    if (measurement
            .pose
            .getTranslation()
            .getDistance(RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation())
        > MAX_POSE_ERROR) {
      confidence = 0;
    }

    if (confidence > 0) {
      hasVision = true;
      currentEstimate = new VisionCameraEstimate(measurement);
      currentConfidence = confidence;
      RobotContainer.poseSensorFusion.addVisionMeasurement(
          currentEstimate.pose,
          currentEstimate.timestampSeconds,
          VecBuilder.fill(
              currentConfidence,
              currentConfidence,
              trust
                  ? Constants.Limelight.ROT_STD_DEV_WHEN_TRUSTING
                  : 9999999) // some influence of limelight pose rotation
          );
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
