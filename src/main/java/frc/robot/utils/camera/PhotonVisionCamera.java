package frc.robot.utils.camera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.VisionCameraEstimate.RawVisionFiducial;
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
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera implements IVisionCamera {

  private int numTags = 0;
  private double confidence = 0;
  private boolean hasVision = false;
  private boolean connected = false;

  private double currentConfidence = 9999999; // large number means less confident
  private VisionCameraEstimate currentEstimate = new VisionCameraEstimate();
  private VisionCameraEstimate unsafeEstimate = new VisionCameraEstimate();

  private String name;

  private final double SINGLE_TAG_CONFIDENCE;
  private final double MULTI_TAG_CONFIDENCE_CLOSE;
  private final double MULTI_TAG_CONFIDENCE_FAR;
  private final double MT_CLOSE_MAX_DIST;

  private final double MAX_POSE_ERROR;

  private CameraType type;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimatorClose;
  private final PhotonPoseEstimator photonEstimatorFar;

  public boolean isConnected() {
    return connected;
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

  public PhotonVisionCamera(String name, CameraType type, Transform3d robotToCamera) {
    this.name = name;
    this.type = type;
    this.SINGLE_TAG_CONFIDENCE = 0.65;
    this.MULTI_TAG_CONFIDENCE_CLOSE = 0.65;
    this.MULTI_TAG_CONFIDENCE_FAR = 0.7;
    this.MT_CLOSE_MAX_DIST = Units.feetToMeters(7);
    this.MAX_POSE_ERROR = 5; // 5 meters

    camera = new PhotonCamera(name);

    photonEstimatorClose =
        new PhotonPoseEstimator(
            Constants.Game.APRILTAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
    photonEstimatorClose.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorFar =
        new PhotonPoseEstimator(
            Constants.Game.APRILTAG_LAYOUT, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
    photonEstimatorFar.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
      SimCameraProperties cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(
          type.getDetectorWidth(), type.getDetectorHeight(), Rotation2d.fromDegrees(type.fov));
      cameraProp.setCalibError(type.pxError, type.pxErrorStdDev);
      cameraProp.setFPS(type.fps);
      cameraProp.setAvgLatencyMs(type.latencyMs);
      cameraProp.setLatencyStdDevMs(type.latencyStdDevMs);

      PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
      cameraSim.enableDrawWireframe(true);
      RobotContainer.visionSim.addCamera(cameraSim, robotToCamera);
    }
  }

  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  public void updateEstimation(boolean trust) {
    confidence = 0;

    photonEstimatorClose.addHeadingData(
        Timer.getFPGATimestamp(),
        new Rotation3d(
            0,
            0,
            RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation().getRadians()));
    photonEstimatorFar.addHeadingData(
        Timer.getFPGATimestamp(),
        new Rotation3d(
            0,
            0,
            RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation().getRadians()));

    Optional<VisionCameraEstimate> measurement_close_opt =
        getEstimatedGlobalPose(photonEstimatorClose, false);
    Optional<VisionCameraEstimate> measurement_far_opt =
        getEstimatedGlobalPose(photonEstimatorFar, true);

    if (!camera.isConnected()) {
      connected = false;
      return;
    } else {
      connected = true;
    }

    if (measurement_close_opt.isEmpty() && measurement_far_opt.isEmpty()) {
      hasVision = false;
      currentConfidence = 9999999;
      confidence = 0;
      return;
    }

    var measurement_close =
        measurement_close_opt.isEmpty() ? measurement_far_opt.get() : measurement_close_opt.get();
    var measurement_far =
        measurement_far_opt.isEmpty() ? measurement_close_opt.get() : measurement_far_opt.get();

    numTags = measurement_close.tagCount;

    if (!DashboardUI.Autonomous.getForceMT1()) {
      if (measurement_close.tagCount > 0 && SimpleMath.isPoseInField(measurement_close.pose)) {
        if (measurement_close.avgTagDist < MT_CLOSE_MAX_DIST) {
          confidence = MULTI_TAG_CONFIDENCE_CLOSE;
        } else {
          confidence = MULTI_TAG_CONFIDENCE_FAR;
          measurement_close = measurement_far;
        }
      }
    } else {
      confidence = MULTI_TAG_CONFIDENCE_CLOSE;
    }

    unsafeEstimate = measurement_close;

    if (measurement_close
            .pose
            .getTranslation()
            .getDistance(RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation())
        > MAX_POSE_ERROR) {
      confidence = 0;
    }

    if (confidence > 0) {
      hasVision = true;
      currentEstimate = measurement_close;
      currentConfidence = confidence;
      RobotContainer.poseSensorFusion.addVisionMeasurement(
          currentEstimate.pose,
          currentEstimate.timestampSeconds,
          VecBuilder.fill(
              currentConfidence,
              currentConfidence,
              trust
                  ? Constants.PhotonVision.ROT_STD_DEV_WHEN_TRUSTING
                  : 9999999) // some influence of limelight pose rotation
          );
    } else {
      hasVision = false;
      currentConfidence = 9999999;
    }
  }

  public void logValues(String id) {
    String prefix = "PhotonCamera/" + id + "/";
    Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose);
    Logger.recordOutput(prefix + "NumTags", numTags);
    Logger.recordOutput(prefix + "Confidence", currentConfidence);
    Logger.recordOutput(prefix + "HasVision", hasVision);
    Logger.recordOutput(prefix + "Connected", connected);
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<VisionCameraEstimate> getEstimatedGlobalPose(
      PhotonPoseEstimator estimator, boolean isConstrained) {
    Optional<VisionCameraEstimate> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      var est_opt = estimator.update(change);

      if (est_opt.isPresent()) {
        var est = est_opt.get();
        int numTags = est.targetsUsed.size();

        double avgTagDist = 0;
        double avgTagArea = 0;

        List<RawVisionFiducial> rawFiducials = new ArrayList<>(numTags);
        for (var target : est.targetsUsed) {
          double dist = target.getBestCameraToTarget().getTranslation().getNorm();
          double distRobot =
              target
                  .getBestCameraToTarget()
                  .plus(estimator.getRobotToCameraTransform())
                  .getTranslation()
                  .getNorm();
          avgTagDist += dist;
          avgTagArea += target.getArea();

          var rawFiducial =
              new RawVisionFiducial(
                  target.fiducialId, target.getArea(), dist, distRobot, target.getPoseAmbiguity());
          rawFiducials.add(rawFiducial);
        }
        avgTagDist /= numTags;
        avgTagArea /= numTags;

        VisionCameraEstimate estimation =
            new VisionCameraEstimate(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                type.latencyMs,
                numTags,
                avgTagDist,
                avgTagArea,
                rawFiducials.toArray(new RawVisionFiducial[0]),
                isConstrained);

        visionEst = Optional.of(estimation);
      } else {
        visionEst = Optional.empty();
      }

      updateEstimationStdDevs(estimator, visionEst, change.getTargets());
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      PhotonPoseEstimator estimator,
      Optional<VisionCameraEstimate> estimatedPose,
      List<PhotonTrackedTarget> targets) {

    double confidence = 0;

    if (!estimatedPose.isEmpty()) {
      // Pose present. Start running Heuristic
      confidence = SINGLE_TAG_CONFIDENCE;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().pose.getTranslation());
      }

      if (numTags == 0) {
        confidence = 0;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) confidence = MULTI_TAG_CONFIDENCE_CLOSE;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) confidence = 0;
        else confidence = confidence * (1 + (avgDist * avgDist / 30));
      }
    }
  }

  public void close() {
    // camera.close();
  }

  public void kill() {}
}
