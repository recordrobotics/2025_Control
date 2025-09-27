package frc.robot.utils.camera;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.VisionSimulationMode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.PoseSensorFusion;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.VisionCameraEstimate.RawVisionFiducial;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera implements IVisionCamera {

    private static final ImmutableList<RawVisionFiducial> ALL_SIM_TAGS =
            ImmutableList.copyOf(Constants.Game.APRILTAG_LAYOUT.getTags().stream()
                    .map(v -> new RawVisionFiducial(v.ID, 0.1, 6, 7, 0))
                    .toList());

    private static final double DEFAULT_CONFIDENCE_CLOSE = 0.65;
    private static final double DEFAULT_CONFIDENCE_FAR = 0.7;
    private static final double DEFAULT_CLOSE_MAX_DISTANCE = Units.feetToMeters(7);
    private static final double DEFAULT_MAX_POSE_ERROR = 10; // 10 meters

    private static final double MAPLE_SIM_STDDEV = 0.001;
    private static final double MAPLE_SIM_TAG_DIST = 6;
    private static final double MAPLE_SIM_TAG_AREA = 0.1;

    private int numTags = 0;
    private boolean hasVision = false;
    private boolean connected = false;

    private double currentMeasurementStdDevs =
            PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS; // large number means less confident
    private VisionCameraEstimate currentEstimate = new VisionCameraEstimate();
    private VisionCameraEstimate unsafeEstimate = new VisionCameraEstimate();

    private String name;

    private final double confidenceClose;
    private final double confidenceFar;
    private final double closeMaxDistance;

    private final double maxPoseError;

    private CameraType type;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimatorClose;
    private final PhotonPoseEstimator photonEstimatorFar;

    public PhotonVisionCamera(String name, CameraType type, Transform3d robotToCamera, double stdMultiplier) {
        this.name = name;
        this.type = type;
        this.confidenceClose = DEFAULT_CONFIDENCE_CLOSE * stdMultiplier;
        this.confidenceFar = DEFAULT_CONFIDENCE_FAR * stdMultiplier;
        this.closeMaxDistance = DEFAULT_CLOSE_MAX_DISTANCE;
        this.maxPoseError = DEFAULT_MAX_POSE_ERROR;

        camera = new PhotonCamera(name);

        photonEstimatorClose = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        photonEstimatorClose.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorFar = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
        photonEstimatorFar.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL
                && Constants.RobotState.VISION_SIMULATION_MODE
                        == Constants.RobotState.VisionSimulationMode.PHOTON_SIM) {
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

    @Override
    public boolean isConnected() {
        return connected;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public CameraType getCameraType() {
        return type;
    }

    @Override
    public boolean hasVision() {
        return hasVision;
    }

    @Override
    public int getNumTags() {
        return numTags;
    }

    @Override
    public VisionCameraEstimate getCurrentEstimate() {
        return currentEstimate;
    }

    @Override
    public VisionCameraEstimate getUnsafeEstimate() {
        return unsafeEstimate;
    }

    @Override
    public double getMeasurementStdDevs() {
        return currentMeasurementStdDevs;
    }

    @Override
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public void updateEstimation(boolean trust, boolean ignore) {
        updateConnectionStatus();
        if (!connected) return;

        MeasurementPair measurements = getMeasurements();

        if (measurements.isEmpty()) {
            resetVisionState();
            return;
        }

        VisionCameraEstimate selectedMeasurement = selectBestMeasurement(measurements);
        updateVisionEstimate(selectedMeasurement, trust);
    }

    private MeasurementPair getMeasurements() {
        if (isRealOrPhotonSim()) {
            return getPhotonMeasurements();
        }
        return getMapleSimMeasurements();
    }

    private static boolean isRealOrPhotonSim() {
        return Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                || Constants.RobotState.VISION_SIMULATION_MODE == Constants.RobotState.VisionSimulationMode.PHOTON_SIM;
    }

    private MeasurementPair getPhotonMeasurements() {
        addHeadingDataToEstimators();

        Optional<VisionCameraEstimate> closeOpt = getEstimatedGlobalPose(photonEstimatorClose, false);
        Optional<VisionCameraEstimate> farOpt = getEstimatedGlobalPose(photonEstimatorFar, true);

        return new MeasurementPair(closeOpt.isEmpty() ? null : closeOpt.get(), farOpt.isEmpty() ? null : farOpt.get());
    }

    private void addHeadingDataToEstimators() {
        double timestamp = Timer.getTimestamp();
        Rotation3d heading = new Rotation3d(
                0,
                0,
                RobotContainer.poseSensorFusion
                        .getEstimatedPosition()
                        .getRotation()
                        .getRadians());

        photonEstimatorClose.addHeadingData(timestamp, heading);
        photonEstimatorFar.addHeadingData(timestamp, heading);
    }

    private MeasurementPair getMapleSimMeasurements() {
        Pose2d maplePose = RobotContainer.model.getRobot();
        if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.MAPLE_NOISE) {
            maplePose = SimpleMath.poseNoise(maplePose, MAPLE_SIM_STDDEV, MAPLE_SIM_STDDEV);
        }

        VisionCameraEstimate closeEstimate = new VisionCameraEstimate(
                maplePose,
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(type.latencyMs),
                ALL_SIM_TAGS.size(),
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA,
                ALL_SIM_TAGS,
                false);
        VisionCameraEstimate farEstimate = new VisionCameraEstimate(
                maplePose,
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(type.latencyMs),
                ALL_SIM_TAGS.size(),
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA,
                ALL_SIM_TAGS,
                true);

        return new MeasurementPair(closeEstimate, farEstimate);
    }

    private void updateConnectionStatus() {
        connected = camera.isConnected();
    }

    private void resetVisionState() {
        hasVision = false;
        currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
    }

    private VisionCameraEstimate selectBestMeasurement(MeasurementPair measurements) {
        VisionCameraEstimate measurement =
                calculateConfidenceAndEstimate(measurements.getCloseMeasurement(), measurements.getFarMeasurement());
        numTags = measurement.tagCount();
        unsafeEstimate = measurement;

        return measurement;
    }

    private VisionCameraEstimate calculateConfidenceAndEstimate(
            VisionCameraEstimate closeEst, VisionCameraEstimate farEst) {
        if (DashboardUI.Autonomous.isForceMT1Pressed()) {
            currentMeasurementStdDevs = confidenceClose;
            return closeEst;
        }

        if (closeEst.tagCount() > 0 && SimpleMath.isInField(closeEst.pose())) {
            if (closeEst.avgTagDist() < closeMaxDistance) {
                currentMeasurementStdDevs = confidenceClose;
            } else {
                currentMeasurementStdDevs = confidenceFar;
                return farEst;
            }
        }

        return closeEst;
    }

    private void updateVisionEstimate(VisionCameraEstimate measurement, boolean trust) {
        if (isPoseErrorTooLarge(measurement)) {
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        }

        if (currentMeasurementStdDevs < PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS) {
            applyVisionMeasurement(measurement, trust);
        } else {
            hasVision = false;
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        }
    }

    private boolean isPoseErrorTooLarge(VisionCameraEstimate measurement) {
        return measurement
                        .pose()
                        .getTranslation()
                        .getDistance(RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation())
                > maxPoseError;
    }

    private void applyVisionMeasurement(VisionCameraEstimate measurement, boolean trust) {
        hasVision = true;
        currentEstimate = measurement;
        RobotContainer.poseSensorFusion.addVisionMeasurement(
                currentEstimate.pose(),
                currentEstimate.timestampSeconds(),
                VecBuilder.fill(
                        currentMeasurementStdDevs,
                        currentMeasurementStdDevs,
                        trust
                                ? Constants.PhotonVision.ROT_STD_DEV_WHEN_TRUSTING
                                : PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS));
    }

    private static final class MeasurementPair {
        private final VisionCameraEstimate closeOpt;
        private final VisionCameraEstimate farOpt;

        private MeasurementPair(VisionCameraEstimate close, VisionCameraEstimate far) {
            this.closeOpt = close;
            this.farOpt = far;
        }

        private boolean isEmpty() {
            return closeOpt == null && farOpt == null;
        }

        private VisionCameraEstimate getCloseMeasurement() {
            return closeOpt == null ? farOpt : closeOpt;
        }

        private VisionCameraEstimate getFarMeasurement() {
            return farOpt == null ? closeOpt : farOpt;
        }
    }

    @Override
    public void logValues(String id) {
        String prefix = "PhotonCamera/" + id + "/";
        Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose());
        Logger.recordOutput(prefix + "NumTags", numTags);
        Logger.recordOutput(prefix + "Confidence", currentMeasurementStdDevs);
        Logger.recordOutput(prefix + "HasVision", hasVision);
        Logger.recordOutput(prefix + "Connected", connected);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<VisionCameraEstimate> getEstimatedGlobalPose(PhotonPoseEstimator estimator, boolean isConstrained) {
        Optional<VisionCameraEstimate> visionEst = Optional.empty();
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estOpt = estimator.update(change);

            if (estOpt.isPresent()) {
                EstimatedRobotPose est = estOpt.get();
                int targetNum = est.targetsUsed.size();

                double avgTagDist = 0;
                double avgTagArea = 0;

                ImmutableList.Builder<RawVisionFiducial> rawFiducials =
                        ImmutableList.builderWithExpectedSize(targetNum);
                for (PhotonTrackedTarget target : est.targetsUsed) {
                    double dist =
                            target.getBestCameraToTarget().getTranslation().getNorm();
                    double distRobot = target.getBestCameraToTarget()
                            .plus(estimator.getRobotToCameraTransform())
                            .getTranslation()
                            .getNorm();
                    avgTagDist += dist;
                    avgTagArea += target.getArea();

                    RawVisionFiducial rawFiducial = new RawVisionFiducial(
                            target.fiducialId, target.getArea(), dist, distRobot, target.getPoseAmbiguity());
                    rawFiducials.add(rawFiducial);
                }
                avgTagDist /= targetNum;
                avgTagArea /= targetNum;

                VisionCameraEstimate estimation = new VisionCameraEstimate(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        type.latencyMs,
                        targetNum,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials.build(),
                        isConstrained);

                visionEst = Optional.of(estimation);
            } else {
                visionEst = Optional.empty();
            }
        }
        return visionEst;
    }
}
