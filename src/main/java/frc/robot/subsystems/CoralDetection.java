package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralDetection extends SubsystemBase {

  // no IO since simulation does not currently support non-apriltag pipelines, so no point
  private PhotonCamera camera;
  // instead simulation can choose between maple sim coral or external photonvision client
  private CoralDetectionSimulationMode simulationMode = CoralDetectionSimulationMode.MAPLE_SIM;

  public enum CoralDetectionSimulationMode {
    PHOTONVISION,
    MAPLE_SIM
  }

  public static class DetectedCoral {
    public Transform3d cameraRelative;
    public double timestamp;

    public DetectedCoral(Transform3d cameraRelative, double timestamp) {
      this.cameraRelative = cameraRelative;
      this.timestamp = timestamp;
    }

    public Pose3d getPose(Pose3d currentCameraPose) {
      return getCoralPose(cameraRelative, currentCameraPose);
    }

    public static Pose3d getCoralPose(Transform3d cameraRelative, Pose3d currentCameraPose) {
      return currentCameraPose.transformBy(cameraRelative);
    }
  }

  record PhotonTrackedTargetTimestamped(PhotonTrackedTarget target, double timestamp) {}

  private List<DetectedCoral> detectedCorals = new ArrayList<>();

  public CoralDetection() {
    camera = new PhotonCamera(Constants.PhotonVision.PHOTON_CORAL_INTAKE);
  }

  @Override
  public void periodic() {
    if (Constants.RobotState.getMode() == Mode.REAL
        || simulationMode == CoralDetectionSimulationMode.PHOTONVISION) {
      List<PhotonTrackedTargetTimestamped> targets = new ArrayList<>();

      if (Constants.RobotState.getMode() == Mode.SIM && !camera.isConnected()) {
        DriverStation.reportError(
            "Coral Detection camera not connected! Did you forget to setSimulationMode(CoralDetectionSimulationMode.MAPLE_SIM)?",
            false);
      }

      for (var result : camera.getAllUnreadResults()) {
        if (result.hasTargets()) {
          targets.addAll(
              result.getTargets().stream()
                  .filter(target -> target.objDetectId == Constants.PhotonVision.CORAL_ID)
                  .map(
                      target ->
                          new PhotonTrackedTargetTimestamped(target, result.getTimestampSeconds()))
                  .toList());
        }
      }

      var robot3d = new Pose3d(RobotContainer.poseSensorFusion.getEstimatedPosition());
      var camera3d = robot3d.transformBy(Constants.PhotonVision.groundIntakeTransformRobotToCamera);

      // Update coral

      var projected = project2dPoses(targets);
      for (int i = 0; i < projected.size(); i++) {
        var projectedField = DetectedCoral.getCoralPose(projected.get(i), camera3d);
        boolean isNew = true;

        for (var coral : detectedCorals) {
          var coralField = coral.getPose(camera3d);
          if (projectedField.getTranslation().getDistance(coralField.getTranslation())
              < Constants.PhotonVision.CORAL_ID_DISTANCE.in(Meters)) {
            isNew = false;
            if (targets.get(i).timestamp > coral.timestamp) {
              coral.cameraRelative = projected.get(i);
              coral.timestamp = targets.get(i).timestamp;
              break;
            }
          }
        }

        if (isNew) {
          detectedCorals.add(new DetectedCoral(projected.get(i), targets.get(i).timestamp));
        }
      }

      // Remove old corals
      detectedCorals.removeIf(
          coral -> {
            double age = Timer.getFPGATimestamp() - coral.timestamp;
            return age > Constants.PhotonVision.CORAL_TIMEOUT.in(Seconds) || age < 0;
          });
    }
  }

  @AutoLogOutput
  public Pose3d[] getCorals() {
    if (Constants.RobotState.getMode() == Mode.REAL
        || simulationMode == CoralDetectionSimulationMode.PHOTONVISION) {
      var robot3d = new Pose3d(RobotContainer.poseSensorFusion.getEstimatedPosition());
      var camera3d = robot3d.transformBy(Constants.PhotonVision.groundIntakeTransformRobotToCamera);

      Pose3d[] corals = new Pose3d[detectedCorals.size()];
      for (int i = 0; i < detectedCorals.size(); i++) {
        DetectedCoral coral = detectedCorals.get(i);
        corals[i] = coral.getPose(camera3d);
      }
      return corals;
    } else {
      return SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]);
    }
  }

  public void setSimulationMode(CoralDetectionSimulationMode mode) {
    simulationMode = mode;
  }

  public CoralDetectionSimulationMode getSimulationMode() {
    return simulationMode;
  }

  /**
   * Uses law of sines assuming a constant height ground plane to project 2d targets into 3d
   *
   * @param targets
   * @return
   */
  private List<Transform3d> project2dPoses(List<PhotonTrackedTargetTimestamped> targets) {
    return targets.stream()
        .map(
            t -> {
              var target = t.target;
              double theta =
                  Math.PI / 2
                      + Constants.PhotonVision.groundIntakeTransformRobotToCamera
                          .getRotation()
                          .getMeasureY()
                          .in(Radians)
                      + Units.degreesToRadians(target.getPitch());

              double forward =
                  Constants.PhotonVision.groundIntakeTransformRobotToCamera
                          .getTranslation()
                          .getMeasureZ()
                          .in(Meters)
                      * Math.tan(theta);

              double minX = Double.MAX_VALUE;
              double maxX = Double.MIN_VALUE;
              double minY = Double.MAX_VALUE;
              double maxY = Double.MIN_VALUE;

              for (var corner : target.minAreaRectCorners) {
                minX = Math.min(minX, corner.x);
                maxX = Math.max(maxX, corner.x);
                minY = Math.min(minY, corner.y);
                maxY = Math.max(maxY, corner.y);
              }

              double width = maxX - minX;
              double height = maxY - minY;
              double aspectRatio = width / height;

              double cameraRelativeYaw =
                  Math.PI / 2 * MathUtil.inverseInterpolate(0.5, 2.5, aspectRatio);

              return new Transform3d(
                  new Translation3d(
                          forward,
                          0,
                          -Constants.PhotonVision.groundIntakeTransformRobotToCamera
                              .getTranslation()
                              .getMeasureZ()
                              .in(Meters))
                      .rotateBy(
                          new Rotation3d(
                              0,
                              -Constants.PhotonVision.groundIntakeTransformRobotToCamera
                                  .getRotation()
                                  .getY(),
                              Units.degreesToRadians(-target.getYaw()))),
                  new Rotation3d(
                          0,
                          0,
                          cameraRelativeYaw
                              + Constants.PhotonVision.groundIntakeTransformRobotToCamera
                                  .getRotation()
                                  .getZ())
                      .rotateBy(
                          Constants.PhotonVision.groundIntakeTransformRobotToCamera
                              .getRotation()
                              .unaryMinus()));
            })
        .toList();
  }
}
