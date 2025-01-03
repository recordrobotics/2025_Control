package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PoseTracker extends SubsystemBase {
  public static PoseTracker instance;

  private final NavSensor nav = new NavSensor();

  private static SwerveDrivePoseEstimator poseFilter;

  private final Drivetrain drivetrain;
  private final Limelight limelight;

  public PoseTracker(Drivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;

    nav.resetAngleAdjustment();

    poseFilter =
        new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            nav.getAdjustedAngle(),
            getModulePositions(),
            ShuffleboardUI.Autonomous.getStartingLocation().getPose());
  }

  @Override
  public void periodic() {
    poseFilter.update(nav.getAdjustedAngle(), getModulePositions());
    poseFilter.addVisionMeasurement(
        limelight.getPoseEstimate().pose,
        limelight.getPoseEstimate().timestampSeconds,
        VecBuilder.fill(
            limelight.getConfidence(),
            limelight.getConfidence(),
            9999999) // big number to remove all influence of limelight pose rotation
        );

    SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
    SmartDashboard.putNumber("pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
    ShuffleboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());
  }

  private SwerveModulePosition[] getModulePositions() {
    return drivetrain.getModulePositions();
  }

  public Pose2d _getEstimatedPosition() {
    return poseFilter.getEstimatedPosition();
  }

  /** Similar to resetPose but adds an argument for the initial pose */
  public void _setToPose(Pose2d pose) {
    poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
  }

  /** Resets the field relative position of the robot (mostly for testing). */
  public void _resetStartingPose() {
    setToPose(ShuffleboardUI.Autonomous.getStartingLocation().getPose());
  }

  /** Resets the pose to FrontSpeakerClose (shooter facing towards speaker) */
  public void _resetDriverPose() {
    poseFilter.resetPosition(
        nav.getAdjustedAngle(),
        getModulePositions(),
        Constants.FieldStartingLocation.FrontSpeakerClose.getPose());
  }

  // Singleton stuff
  // just static versions of the above methods to avoid .instance boilerplate

  public static Pose2d getEstimatedPosition() {
    return PoseTracker.instance._getEstimatedPosition();
  }

  public static void setToPose(Pose2d pose) {
    PoseTracker.instance._setToPose(pose);
  }

  public static void resetStartingPose() {
    PoseTracker.instance._resetStartingPose();
  }

  public static void resetDriverPose() {
    PoseTracker.instance._resetDriverPose();
  }
}
