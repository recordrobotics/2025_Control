package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.real.NavSensorReal;
import frc.robot.subsystems.io.sim.NavSensorSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.IndependentSwervePoseEstimator;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.camera.CameraType;
import frc.robot.utils.camera.IVisionCamera;
import frc.robot.utils.camera.LimelightCamera;
import frc.robot.utils.camera.PhotonVisionCamera;
import frc.robot.utils.camera.VisionCameraEstimate.RawVisionFiducial;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;

public class PoseSensorFusion extends SubsystemBase
    implements AutoCloseable, ShuffleboardPublisher {

  public final NavSensor nav;

  private static SwerveDrivePoseEstimator poseFilter;
  private static IndependentSwervePoseEstimator independentPoseEstimator;

  private LimelightCamera leftCamera =
      new LimelightCamera(
          Constants.Limelight.LIMELIGHT_LEFT_NAME,
          CameraType.Limelight3G,
          Constants.Limelight.leftTransformRobotToCamera);
  private LimelightCamera centerCamera =
      new LimelightCamera(
          Constants.Limelight.LIMELIGHT_CENTER_NAME,
          CameraType.Limelight2,
          Constants.Limelight.centerTransformRobotToCamera);

  private PhotonVisionCamera l1Camera =
      new PhotonVisionCamera(
          Constants.PhotonVision.PHOTON_L1_NAME,
          CameraType.SVPROGlobalShutter,
          Constants.PhotonVision.l1TransformRobotToCamera);
  private PhotonVisionCamera sourceCamera =
      new PhotonVisionCamera(
          Constants.PhotonVision.PHOTON_SOURCE_NAME,
          CameraType.SVPROGlobalShutter,
          Constants.PhotonVision.sourceTransformRobotToCamera);

  private final HashSet<VisionDebouncer> visionDebouncers = new HashSet<>();

  public PoseSensorFusion() {
    nav =
        new NavSensor(
            Constants.RobotState.getMode() == Mode.REAL
                ? new NavSensorReal()
                : new NavSensorSim(
                    RobotContainer.drivetrain.getSwerveDriveSimulation().getGyroSimulation()));
    nav.resetAngleAdjustment();

    poseFilter =
        new SwerveDrivePoseEstimator(
            RobotContainer.drivetrain.getKinematics(),
            nav.getAdjustedAngle(),
            getModulePositions(),
            DashboardUI.Autonomous.getStartingLocation().getPose());

    independentPoseEstimator =
        new IndependentSwervePoseEstimator(
            getEstimatedPosition(),
            new SwerveModule[] {
              RobotContainer.drivetrain.getFrontLeftModule(),
              RobotContainer.drivetrain.getFrontRightModule(),
              RobotContainer.drivetrain.getBackLeftModule(),
              RobotContainer.drivetrain.getBackRightModule()
            },
            new Translation2d[] {
              Constants.Swerve.frontLeftConstants.wheelLocation,
              Constants.Swerve.frontRightConstants.wheelLocation,
              Constants.Swerve.backLeftConstants.wheelLocation,
              Constants.Swerve.backRightConstants.wheelLocation
            });

    SmartDashboard.putBoolean("Autonomous/TrustLimelightLeft", false);
    SmartDashboard.putBoolean("Autonomous/TrustLimelightCenter", false);
    SmartDashboard.putBoolean("Autonomous/UseISPE", true);
  }

  public record DeferredPoseEstimation(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}

  private ConcurrentSkipListSet<DeferredPoseEstimation> deferredPoseEstimations =
      new ConcurrentSkipListSet<>((a, b) -> Double.compare(a.timestampSeconds, b.timestampSeconds));

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    deferredPoseEstimations.add(
        new DeferredPoseEstimation(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
  }

  private boolean trustLimelightLeft;
  private boolean trustLimelightCenter;
  private boolean useISPE;

  private ExecutorService executor = Executors.newSingleThreadExecutor();
  private Future<?> calculationFuture = null;

  public void startCalculation() {
    deferredPoseEstimations.clear();

    trustLimelightLeft = SmartDashboard.getBoolean("Autonomous/TrustLimelightLeft", false);
    trustLimelightCenter = SmartDashboard.getBoolean("Autonomous/TrustLimelightCenter", false);
    useISPE = SmartDashboard.getBoolean("Autonomous/UseISPE", true);

    calculationFuture = executor.submit(this::calculationLoop);
  }

  private double updateTimestamp;
  private Rotation2d updateNav;
  private SwerveModulePosition[] updatePositions;

  public void endCalculation() {
    if (updatePositions != null) {
      poseFilter.updateWithTime(updateTimestamp, updateNav, updatePositions);
    }

    if (calculationFuture != null) {
      try {
        calculationFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        e.printStackTrace();
      }
    }

    while (!deferredPoseEstimations.isEmpty()) {
      DeferredPoseEstimation estimation = deferredPoseEstimations.pollFirst();
      poseFilter.addVisionMeasurement(
          estimation.visionRobotPoseMeters,
          estimation.timestampSeconds,
          estimation.visionMeasurementStdDevs);
    }

    updateDashboard(leftCamera, centerCamera, l1Camera, sourceCamera);

    leftCamera.logValues("Left");
    centerCamera.logValues("Center");
    l1Camera.logValues("L1");
    sourceCamera.logValues("Source");

    for (VisionDebouncer debouncer : visionDebouncers) {
      debouncer.update();
    }

    Logger.recordOutput(
        "SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
    Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
    Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());
  }

  @Override
  public void periodic() {}

  public void calculationLoop() {
    updateTimestamp = Timer.getFPGATimestamp();
    updateNav = nav.getAdjustedAngle();
    updatePositions = getModulePositions();

    if (leftCamera.hasVision() || centerCamera.hasVision()) {
      // when vision is correcting the pose, have that override the independent pose estimator
      independentPoseEstimator.reset(getEstimatedPosition());
      independentPoseEstimator.update(getEstimatedPosition().getRotation());
    } else {
      independentPoseEstimator.update(getEstimatedPosition().getRotation());

      // when no vision use independent pose estimator to correct pose
      if (useISPE) {
        addVisionMeasurement(
            independentPoseEstimator.getEstimatedRobotPose(),
            Timer.getFPGATimestamp(),
            VecBuilder.fill(0.7, 0.7, 9999999));
      }
    }

    leftCamera.updateEstimation(trustLimelightLeft);
    centerCamera.updateEstimation(trustLimelightCenter);
    l1Camera.updateEstimation(false);
    sourceCamera.updateEstimation(false);
  }

  private void updateDashboard(IVisionCamera... cameras) {
    for (IVisionCamera camera : cameras) {
      DashboardUI.Autonomous.setVisionPose(camera.getName(), camera.getUnsafeEstimate().pose);
    }

    SmartDashboard.putNumber("pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
    DashboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());
  }

  private SwerveModulePosition[] getModulePositions() {
    return RobotContainer.drivetrain.getModulePositions();
  }

  @AutoLogLevel(key = "Odometry/Robot", level = Level.Real)
  public Pose2d getEstimatedPosition() {
    return poseFilter.getEstimatedPosition();
  }

  public Pose2d getEstimatedPositionAt(double timestamp) {
    var sample = poseFilter.sampleAt(timestamp);
    if (sample.isEmpty()) return getEstimatedPosition();
    else return sample.get();
  }

  /** Similar to resetPose but adds an argument for the initial pose */
  public void setToPose(Pose2d pose) {
    poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
    independentPoseEstimator.reset(pose);
  }

  /** Resets the field relative position of the robot (mostly for testing). */
  public void resetStartingPose() {
    setToPose(DashboardUI.Autonomous.getStartingLocation().getPose());
    if (Constants.RobotState.getMode() == Constants.RobotState.Mode.SIM) {
      RobotContainer.drivetrain
          .getSwerveDriveSimulation()
          .setSimulationWorldPose(DashboardUI.Autonomous.getStartingLocation().getPose());
    }
  }

  public void resetToLimelight() {
    setToPose(
        new Pose2d(
            getEstimatedPosition().getTranslation(),
            leftCamera.getCurrentEstimate().pose.getRotation()));
  }

  public void resetFullLimelight() {
    Translation2d translation;

    if (leftCamera.hasVision()) {
      translation = leftCamera.getCurrentEstimate().pose.getTranslation();
    } else if (centerCamera.hasVision()) {
      translation = centerCamera.getCurrentEstimate().pose.getTranslation();
    } else {
      translation = leftCamera.getCurrentEstimate().pose.getTranslation();
    }

    setToPose(new Pose2d(translation, getEstimatedPosition().getRotation()));
  }

  /**
   * Resets the pose to face elevator away from driverstation, while keeping translation the same
   */
  public void resetDriverPose() {
    Pose2d current = getEstimatedPosition();
    setToPose(
        new Pose2d(
            current.getTranslation(),
            DriverStationUtils.getCurrentAlliance() == Alliance.Red
                ? new Rotation2d(Math.PI)
                : new Rotation2d(0)));
  }

  public enum CameraTarget {
    Left,
    Center,
    All;

    boolean contains(CameraTarget camera) {
      return this == camera || this == All;
    }
  }

  public VisionDebouncer registerVisionCheck(CameraTarget camera) {
    return registerVisionCheck(camera, null);
  }

  public VisionDebouncer registerVisionCheck(
      CameraTarget camera, double debounceTime, Debouncer.DebounceType debounceType) {
    return registerVisionCheck(camera, null, debounceTime, debounceType);
  }

  public VisionDebouncer registerVisionCheck(CameraTarget camera, int... tagIds) {
    return registerVisionCheck(camera, tagIds, 0.5, Debouncer.DebounceType.kBoth);
  }

  private int nextVisionId = 1;

  public VisionDebouncer registerVisionCheck(
      CameraTarget camera, int[] tagIds, double debounceTime, Debouncer.DebounceType debounceType) {
    VisionDebouncer vision =
        new VisionDebouncer(nextVisionId, debounceTime, debounceType, camera, tagIds);
    visionDebouncers.add(vision);
    return vision;
  }

  public void releaseVisionCheck(VisionDebouncer visionDebouncer) {
    visionDebouncers.remove(visionDebouncer);
  }

  public void close() throws Exception {
    nav.close();
  }

  public LimelightCamera getLeftCamera() {
    return leftCamera;
  }

  public LimelightCamera getCenterCamera() {
    return centerCamera;
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Overview.setTagNumLeft(() -> leftCamera.getNumTags());
    DashboardUI.Overview.setConfidenceLeft(() -> leftCamera.getUnprocessedConfidence());
    DashboardUI.Overview.setHasVisionLeft(() -> leftCamera.hasVision());
    DashboardUI.Overview.setLimelightConnectedLeft(() -> leftCamera.isConnected());

    DashboardUI.Overview.setTagNumCenter(() -> centerCamera.getNumTags());
    DashboardUI.Overview.setConfidenceCenter(() -> centerCamera.getUnprocessedConfidence());
    DashboardUI.Overview.setHasVisionCenter(() -> centerCamera.hasVision());
    DashboardUI.Overview.setLimelightConnectedCenter(() -> centerCamera.isConnected());
  }

  public class VisionDebouncer {
    private final Debouncer debouncer;
    private CameraTarget camera;
    private int[] tagIds;

    private final int id;

    private boolean result = false;
    private double lastAccessTime;

    private VisionDebouncer(
        int id,
        double debounceTime,
        Debouncer.DebounceType debounceType,
        CameraTarget camera,
        int[] tagIds) {
      this.id = id;
      this.camera = camera;
      this.tagIds = tagIds;
      debouncer = new Debouncer(debounceTime, debounceType);
      lastAccessTime = Timer.getTimestamp();
    }

    private void update() {
      boolean rawInput = false;

      if (camera.contains(CameraTarget.Left)) {
        rawInput |= RobotContainer.poseSensorFusion.getLeftCamera().hasVision();
      }

      if (camera.contains(CameraTarget.Center)) {
        rawInput |= RobotContainer.poseSensorFusion.getCenterCamera().hasVision();
      }

      if (tagIds != null && rawInput) {
        ArrayList<Integer> visionTags = new ArrayList<>();
        if (camera.contains(CameraTarget.Left)) {
          for (RawVisionFiducial tag :
              RobotContainer.poseSensorFusion.getLeftCamera().getCurrentEstimate().rawFiducials) {
            visionTags.add(tag.id);
          }
        }

        if (camera.contains(CameraTarget.Center)) {
          for (RawVisionFiducial tag :
              RobotContainer.poseSensorFusion.getCenterCamera().getCurrentEstimate().rawFiducials) {
            visionTags.add(tag.id);
          }
        }

        for (int tagId : tagIds) {
          if (!visionTags.contains(tagId)) {
            rawInput = false;
            break;
          }
        }
      }

      result = debouncer.calculate(rawInput);

      double timeSinceLastAccessed = Timer.getTimestamp() - lastAccessTime;
      if (timeSinceLastAccessed > 1.0) {
        DriverStation.reportWarning(
            "Detected abandoned "
                + toString()
                + " last accessed "
                + (int) Math.floor(timeSinceLastAccessed)
                + " seconds ago.",
            false);
      }
    }

    @Override
    public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append("VisionDebouncer{");
      sb.append("id=").append(id);
      sb.append(", camera=").append(camera);
      sb.append(", tagIds=");
      if (tagIds != null) {
        for (int tagId : tagIds) {
          sb.append(tagId).append(" ");
        }
      }
      sb.append(", value=").append(result);
      sb.append('}');
      return sb.toString();
    }

    public boolean hasVision() {
      lastAccessTime = Timer.getTimestamp();
      return result;
    }

    public void setCamera(CameraTarget camera) {
      this.camera = camera;
    }

    public void setTagIds(int[] tagIds) {
      this.tagIds = tagIds;
    }

    public CameraTarget getCamera() {
      return camera;
    }

    public int[] getTagIds() {
      return tagIds;
    }

    public void setDebounceTime(double time) {
      debouncer.setDebounceTime(time);
    }

    public void setDebounceType(Debouncer.DebounceType debounceType) {
      debouncer.setDebounceType(debounceType);
    }

    public double getDebounceTime() {
      return debouncer.getDebounceTime();
    }

    public Debouncer.DebounceType getDebounceType() {
      return debouncer.getDebounceType();
    }

    public int getId() {
      return id;
    }

    public void release() {
      RobotContainer.poseSensorFusion.releaseVisionCheck(this);
    }
  }
}
