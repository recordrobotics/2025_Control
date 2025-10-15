package frc.robot.utils.camera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.VisionSimulationMode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.PoseSensorFusion;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.VisionCameraEstimate.TXTYMeasurement;
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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class LimelightCamera implements IVisionCamera {

    private static final RawFiducial[] ALL_SIM_TAGS = Constants.Game.APRILTAG_LAYOUT.getTags().stream()
            .map(v -> new RawFiducial(v.ID, 0.1, 0.1, 0.1, 6, 7, 0))
            .toArray(RawFiducial[]::new);

    private static final double DEFAULT_CONFIDENCE_MT1 = 0.65;
    private static final double DEFAULT_CONFIDENCE_MT2 = 0.7;
    private static final double DEFAULT_CLOSE_MT1_DISTANCE =
            Units.feetToMeters(7); // 7 feet is where the MT1 (yellow) gets bad wiggles
    private static final double DEFAULT_MAX_POSE_ERROR = 10; // 10 meters
    private static final double DEFAULT_TXTY_MAX_DISTANCE = 0.5; // have to be 0.5 meters or closer to use txty

    private static final double MAPLE_SIM_STDDEV = 0.001;
    private static final double MAPLE_SIM_TAG_SPAN = 0.1;
    private static final double MAPLE_SIM_TAG_DIST = 6;
    private static final double MAPLE_SIM_TAG_AREA = 2.0;

    private int numTags = 0;
    private boolean hasVision = false;
    private boolean limelightConnected = false;

    private double currentMeasurementStdDevs =
            PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS; // large number means less confident
    private VisionCameraEstimate currentEstimate = new VisionCameraEstimate();
    private VisionCameraEstimate unsafeEstimate = new VisionCameraEstimate();

    private String name;

    private final double mt1Confidence;
    private final double mt2Confidence;

    private final double mt1MaxDistance;
    private final double txtyMaxDistance;

    private final double maxPoseError;

    private final Transform3d robotToCamera;

    private PhotonCamera fakeCamera;
    private PhotonPoseEstimator photonEstimator;
    private PhotonPoseEstimator photonEstimatorTXTY;
    private CameraType type;

    private Pose2d txtyPose = new Pose2d();

    public LimelightCamera(String name, CameraType type, Transform3d robotToCamera) {
        this.name = name;
        this.type = type;
        this.robotToCamera = robotToCamera;

        this.mt1Confidence = DEFAULT_CONFIDENCE_MT1;
        this.mt2Confidence = DEFAULT_CONFIDENCE_MT2;
        this.mt1MaxDistance = DEFAULT_CLOSE_MT1_DISTANCE;
        this.txtyMaxDistance = DEFAULT_TXTY_MAX_DISTANCE;
        this.maxPoseError = DEFAULT_MAX_POSE_ERROR;

        if (isPhotonSimMode()) {
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

            photonEstimator = new PhotonPoseEstimator(
                    Constants.Game.APRILTAG_LAYOUT, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
            photonEstimatorTXTY = new PhotonPoseEstimator(
                    Constants.Game.APRILTAG_LAYOUT, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera);
        }
    }

    @Override
    public boolean isConnected() {
        return limelightConnected;
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
        LimelightHelpers.setPipelineIndex(name, pipeline);
    }

    @Override
    public void updateEstimation(boolean trust, boolean ignore) {
        updateRobotOrientation();

        Measurements measurements = getMeasurements();

        if (!validateMeasurements(measurements.mt1(), measurements.mt2())) {
            return;
        }

        if (measurements.txty().isEmpty()) {
            txtyPose = new Pose2d();
        } else {
            txtyPose = measurements.txty().get().pose();
        }

        selectBestMeasurement(measurements.mt1(), measurements.mt2(), measurements.txty());
        validateMeasurementTagCount();
        validatePoseDistance();
        processFinalEstimate(trust, ignore);
    }

    private void updateRobotOrientation() {
        Rotation2d estimatedRotation =
                RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation();

        double yawRate = RobotContainer.poseSensorFusion.nav.getYawRate();

        LimelightHelpers.SetRobotOrientation(
                name,
                estimatedRotation.getDegrees(),
                0, // TODO: try out `yawRate`,
                0,
                0,
                0,
                0);

        if (isPhotonSimMode()) {
            double timestamp = Timer.getTimestamp();
            Rotation3d heading = new Rotation3d(0, 0, estimatedRotation.getRadians());

            photonEstimator.addHeadingData(timestamp, heading);
            photonEstimatorTXTY.addHeadingData(timestamp, heading);
        }
    }

    private Measurements getMeasurements() {
        if (isPhotonSimMode()) {
            Optional<Measurements> photonMeasurements = getPhotonSimMeasurements();
            if (photonMeasurements.isPresent()) {
                return photonMeasurements.get();
            }
        } else if (isMapleSimMode()) {
            return getMapleSimMeasurements();
        }

        PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        PoseEstimate measurementM2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        Optional<TXTYMeasurement> txtyMeasurement = measurement == null
                ? Optional.empty()
                : pnpDistanceTrigSolveStrategy(measurement.timestampSeconds, measurement.tagCount);

        return new Measurements(measurement, measurementM2, txtyMeasurement);
    }

    private Optional<TXTYMeasurement> pnpDistanceTrigSolveStrategy(double timestamp, int tagCount) {
        if (tagCount == 0) {
            return Optional.empty();
        }

        Optional<Pose2d> headingSampleOpt = RobotContainer.poseSensorFusion.getEstimatedPositionAt(timestamp);
        if (headingSampleOpt.isEmpty()) {
            return Optional.empty();
        }
        Rotation2d headingSample = headingSampleOpt.get().getRotation();

        Pose3d bestCameraToTarget = LimelightHelpers.toPose3D(LimelightHelpers.getTargetPose_CameraSpace(name));

        Translation2d camToTagTranslation = new Translation3d(
                        bestCameraToTarget.getTranslation().getNorm(),
                        new Rotation3d(
                                0,
                                -Math.toRadians(LimelightHelpers.getTY(name)),
                                -Math.toRadians(LimelightHelpers.getTX(name))))
                .rotateBy(robotToCamera.getRotation())
                .toTranslation2d()
                .rotateBy(headingSample);

        Optional<Pose3d> tagPoseOpt =
                Constants.Game.APRILTAG_LAYOUT.getTagPose((int) LimelightHelpers.getFiducialID(name));
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        Pose2d tagPose2d = tagPoseOpt.get().toPose2d();

        Translation2d fieldToCameraTranslation = tagPose2d.getTranslation().plus(camToTagTranslation.unaryMinus());

        Translation2d camToRobotTranslation =
                robotToCamera.getTranslation().toTranslation2d().unaryMinus().rotateBy(headingSample);

        Pose2d robotPose = new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);

        return Optional.of(new TXTYMeasurement(
                robotPose, timestamp, bestCameraToTarget.getTranslation().getNorm()));
    }

    private static boolean isPhotonSimMode() {
        return Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL
                && Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM;
    }

    private static boolean isMapleSimMode() {
        return Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL && !isPhotonSimMode();
    }

    private Optional<Measurements> getPhotonSimMeasurements() {
        List<PhotonPipelineResult> results = fakeCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        Optional<EstimatedRobotPose> visionEstTXTY = Optional.empty();
        double bestTargetDist = Double.MAX_VALUE;
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            visionEst = photonEstimator.update(result);
            visionEstTXTY = photonEstimatorTXTY.update(result);
            bestTargetDist = result.getBestTarget()
                    .getBestCameraToTarget()
                    .getTranslation()
                    .getNorm();
        }

        final double bestTargetDistFinal = bestTargetDist;

        if (visionEst.isPresent()) {
            return Optional.of(createPoseEstimatesFromPhoton(
                    visionEst.get(),
                    visionEstTXTY.map(v ->
                            new TXTYMeasurement(v.estimatedPose.toPose2d(), v.timestampSeconds, bestTargetDistFinal))));
        }

        return Optional.empty();
    }

    private Measurements createPoseEstimatesFromPhoton(EstimatedRobotPose est, Optional<TXTYMeasurement> txty) {
        PhotonMeasurementData data = processPhotonTargets(est);

        PoseEstimate measurement = new PoseEstimate(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                type.latencyMs,
                data.targetNum,
                data.span,
                data.avgTagDist,
                data.avgTagArea,
                data.rawFiducials.toArray(new RawFiducial[0]),
                false);

        PoseEstimate measurementM2 = new PoseEstimate(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                type.latencyMs,
                data.targetNum,
                data.span,
                data.avgTagDist,
                data.avgTagArea,
                data.rawFiducials.toArray(new RawFiducial[0]),
                true);

        return new Measurements(measurement, measurementM2, txty);
    }

    private PhotonMeasurementData processPhotonTargets(EstimatedRobotPose est) {
        PhotonMeasurementData data = new PhotonMeasurementData();
        data.targetNum = est.targetsUsed.size();
        data.rawFiducials = new ArrayList<>(data.targetNum);

        for (PhotonTrackedTarget target : est.targetsUsed) {
            processPhotonTarget(target, data);
        }

        data.avgTagDist /= data.targetNum;
        data.avgTagArea /= data.targetNum;
        data.span = Math.sqrt((data.maxX - data.minX) * (data.maxY - data.minY));

        return data;
    }

    private void processPhotonTarget(PhotonTrackedTarget target, PhotonMeasurementData data) {
        double dist = target.getBestCameraToTarget().getTranslation().getNorm();
        double distRobot = target.getBestCameraToTarget()
                .plus(photonEstimator.getRobotToCameraTransform())
                .getTranslation()
                .getNorm();

        data.avgTagDist += dist;
        data.avgTagArea += target.getArea();

        double[] centerPoint = calculateTargetCenter(target, data);

        RawFiducial rawFiducial = new RawFiducial(
                target.fiducialId,
                centerPoint[0],
                centerPoint[1],
                target.getArea(),
                dist,
                distRobot,
                target.getPoseAmbiguity());
        data.rawFiducials.add(rawFiducial);
    }

    private static double[] calculateTargetCenter(PhotonTrackedTarget target, PhotonMeasurementData data) {
        double tcx = 0;
        double tcy = 0;

        for (TargetCorner corner : target.getDetectedCorners()) {
            tcx += corner.x;
            tcy += corner.y;

            data.minX = Math.min(data.minX, corner.x);
            data.maxX = Math.max(data.maxX, corner.x);
            data.minY = Math.min(data.minY, corner.y);
            data.maxY = Math.max(data.maxY, corner.y);
        }

        tcx /= target.getDetectedCorners().size();
        tcy /= target.getDetectedCorners().size();

        return new double[] {tcx, tcy};
    }

    private Measurements getMapleSimMeasurements() {
        Pose2d maplePose = RobotContainer.model.getRobot();
        if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.MAPLE_NOISE) {
            maplePose = SimpleMath.poseNoise(maplePose, MAPLE_SIM_STDDEV, MAPLE_SIM_STDDEV);
        }

        PoseEstimate measurement = new PoseEstimate(
                maplePose,
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(type.latencyMs),
                ALL_SIM_TAGS.length,
                MAPLE_SIM_TAG_SPAN,
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA,
                ALL_SIM_TAGS,
                false);

        PoseEstimate measurementM2 = new PoseEstimate(
                maplePose,
                Timer.getTimestamp(),
                Units.millisecondsToSeconds(type.latencyMs),
                ALL_SIM_TAGS.length,
                MAPLE_SIM_TAG_SPAN,
                MAPLE_SIM_TAG_DIST,
                MAPLE_SIM_TAG_AREA,
                ALL_SIM_TAGS,
                true);

        return new Measurements(
                measurement,
                measurementM2,
                Optional.of(new TXTYMeasurement(maplePose, Timer.getTimestamp(), MAPLE_SIM_TAG_DIST)));
    }

    private boolean validateMeasurements(PoseEstimate measurement, PoseEstimate measurementM2) {
        if (measurement == null || measurementM2 == null) {
            if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
                limelightConnected = false;
            }
            numTags = 0;
            return false;
        }
        limelightConnected = true;
        numTags = measurement.tagCount;
        return true;
    }

    private void selectBestMeasurement(
            PoseEstimate measurement, PoseEstimate measurementM2, Optional<TXTYMeasurement> txty) {
        if (DashboardUI.Autonomous.isForceMT1Pressed()
                && measurement.tagCount > 0
                && SimpleMath.isInField(measurement.pose)) {
            currentMeasurementStdDevs = mt1Confidence;
            unsafeEstimate = new VisionCameraEstimate(measurement);
        } else if (measurement.tagCount > 0) {
            if (txty.isPresent()
                    && SimpleMath.isInField(txty.get().pose())
                    && txty.get().distToCamera() <= txtyMaxDistance) {
                currentMeasurementStdDevs = mt1Confidence;
                unsafeEstimate = new VisionCameraEstimate(txty.get());
            } else if (measurement.avgTagDist < mt1MaxDistance && SimpleMath.isInField(measurement.pose)) {
                currentMeasurementStdDevs = mt1Confidence;
                unsafeEstimate = new VisionCameraEstimate(measurement);
            } else if (SimpleMath.isInField(measurementM2.pose)) {
                currentMeasurementStdDevs = mt2Confidence;
                unsafeEstimate = new VisionCameraEstimate(measurementM2);
            } else {
                currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
                unsafeEstimate = new VisionCameraEstimate(measurement);
            }
        } else {
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
            unsafeEstimate = new VisionCameraEstimate(measurement);
        }
    }

    private void validatePoseDistance() {
        if (unsafeEstimate
                                .pose()
                                .getTranslation()
                                .getDistance(RobotContainer.poseSensorFusion
                                        .getEstimatedPosition()
                                        .getTranslation())
                        > maxPoseError
                || !SimpleMath.isInField(unsafeEstimate.pose())) {
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        }
    }

    private void validateMeasurementTagCount() {
        if (unsafeEstimate.tagCount() == 0) {
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        }
    }

    private void processFinalEstimate(boolean trust, boolean ignore) {
        if (currentMeasurementStdDevs < PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS) {
            hasVision = true;
            currentEstimate = unsafeEstimate;
            if (!ignore) {
                addVisionMeasurement(trust);
            }
        } else {
            hasVision = false;
            currentMeasurementStdDevs = PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS;
        }
    }

    private void addVisionMeasurement(boolean trust) {
        RobotContainer.poseSensorFusion.addVisionMeasurement(
                currentEstimate.pose(),
                currentEstimate.timestampSeconds(),
                VecBuilder.fill(
                        currentMeasurementStdDevs,
                        currentMeasurementStdDevs,
                        trust
                                ? Constants.Limelight.ROT_STD_DEV_WHEN_TRUSTING
                                : PoseSensorFusion.MAX_MEASUREMENT_STD_DEVS));
    }

    private static class PhotonMeasurementData {
        int targetNum;
        double avgTagDist = 0;
        double avgTagArea = 0;
        double minX = 1;
        double minY = 1;
        double maxX = -1;
        double maxY = -1;
        double span;
        List<RawFiducial> rawFiducials;
    }

    private record Measurements(PoseEstimate mt1, PoseEstimate mt2, Optional<TXTYMeasurement> txty) {}

    @Override
    public void logValues(String id) {
        String prefix = "Limelight/" + id + "/";
        Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose());
        Logger.recordOutput(prefix + "NumTags", numTags);
        Logger.recordOutput(prefix + "Confidence", currentMeasurementStdDevs);
        Logger.recordOutput(prefix + "HasVision", hasVision);
        Logger.recordOutput(prefix + "LimelightConnected", limelightConnected);
        Logger.recordOutput(prefix + "TXTY", txtyPose);
    }
}
