package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
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
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.IndependentSwervePoseEstimator;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.camera.CameraType;
import frc.robot.utils.camera.IVisionCamera;
import frc.robot.utils.camera.LimelightCamera;
import frc.robot.utils.camera.PhotonVisionCamera;
import org.littletonrobotics.junction.AutoLogOutput;
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

  @Override
  public void periodic() {
    boolean trustLimelightLeft = SmartDashboard.getBoolean("Autonomous/TrustLimelightLeft", false);
    boolean trustLimelightCenter =
        SmartDashboard.getBoolean("Autonomous/TrustLimelightCenter", false);
    boolean useISPE = SmartDashboard.getBoolean("Autonomous/UseISPE", true);

    poseFilter.update(nav.getAdjustedAngle(), getModulePositions());

    if (leftCamera.hasVision() || centerCamera.hasVision()) {
      // when vision is correcting the pose, have that override the independent pose estimator
      independentPoseEstimator.reset(getEstimatedPosition());
      independentPoseEstimator.update(getEstimatedPosition().getRotation());
    } else {
      independentPoseEstimator.update(getEstimatedPosition().getRotation());

      // when no vision use independent pose estimator to correct pose
      if (useISPE) {
        poseFilter.addVisionMeasurement(
            independentPoseEstimator.getEstimatedRobotPose(),
            Timer.getFPGATimestamp(),
            VecBuilder.fill(0.7, 0.7, 9999999));
      }
    }

    leftCamera.updateEstimation(trustLimelightLeft);
    centerCamera.updateEstimation(trustLimelightCenter);
    l1Camera.updateEstimation(false);
    sourceCamera.updateEstimation(false);

    updateDashboard(leftCamera, centerCamera, l1Camera, sourceCamera);

    leftCamera.logValues("Left");
    centerCamera.logValues("Center");
    l1Camera.logValues("L1");
    sourceCamera.logValues("Source");

    Logger.recordOutput(
        "SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
    Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
    Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());
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

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Vector<N3> confidence) {
    poseFilter.addVisionMeasurement(pose, timestampSeconds, confidence);
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getEstimatedPosition() {
    return poseFilter.getEstimatedPosition();
  }

  /** Similar to resetPose but adds an argument for the initial pose */
  public void setToPose(Pose2d pose) {
    poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
    independentPoseEstimator.reset(pose);
  }

  /** Resets the field relative position of the robot (mostly for testing). */
  public void resetStartingPose() {
    setToPose(DashboardUI.Autonomous.getStartingLocation().getPose());
    RobotContainer.drivetrain
        .getSwerveDriveSimulation()
        .setSimulationWorldPose(DashboardUI.Autonomous.getStartingLocation().getPose());
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
}
