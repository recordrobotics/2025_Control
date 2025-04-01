package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PoseTracker extends SubsystemBase implements AutoCloseable {

  public final NavSensor nav =
      new NavSensor(
          Constants.RobotState.getMode() == Mode.REAL ? new NavSensorReal() : new NavSensorSim());

  private static SwerveDrivePoseEstimator poseFilter;
  private static IndependentSwervePoseEstimator independentPoseEstimator;

  public PoseTracker() {
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

    if (RobotContainer.limelight.getLeft().hasVision
        || RobotContainer.limelight.getCenter().hasVision) {
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

    poseFilter.addVisionMeasurement(
        RobotContainer.limelight.getLeft().currentEstimate.pose,
        RobotContainer.limelight.getLeft().currentEstimate.timestampSeconds,
        VecBuilder.fill(
            RobotContainer.limelight.getLeft().currentConfidence,
            RobotContainer.limelight.getLeft().currentConfidence,
            trustLimelightLeft ? 4 : 9999999) // some influence of limelight pose rotation
        );

    poseFilter.addVisionMeasurement(
        RobotContainer.limelight.getCenter().currentEstimate.pose,
        RobotContainer.limelight.getCenter().currentEstimate.timestampSeconds,
        VecBuilder.fill(
            RobotContainer.limelight.getCenter().currentConfidence,
            RobotContainer.limelight.getCenter().currentConfidence,
            trustLimelightCenter ? 4 : 9999999) // some influence of limelight pose rotation
        );

    SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
    SmartDashboard.putNumber("pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
    DashboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());

    Logger.recordOutput(
        "SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
    Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
    Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());
  }

  private SwerveModulePosition[] getModulePositions() {
    return RobotContainer.drivetrain.getModulePositions();
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
  }

  public void resetToLimelight() {
    setToPose(
        new Pose2d(
            getEstimatedPosition().getTranslation(),
            RobotContainer.limelight.getLeft().currentEstimate.pose.getRotation()));
  }

  public void resetFullLimelight() {
    Translation2d translation;

    if (RobotContainer.limelight.getLeft().hasVision) {
      translation = RobotContainer.limelight.getLeft().currentEstimate.pose.getTranslation();
    } else if (RobotContainer.limelight.getCenter().hasVision) {
      translation = RobotContainer.limelight.getCenter().currentEstimate.pose.getTranslation();
    } else {
      translation = RobotContainer.limelight.getLeft().currentEstimate.pose.getTranslation();
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
}
