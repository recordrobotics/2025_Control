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
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ManagedSubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralDetection extends ManagedSubsystemBase {

  // no IO since simulation does not currently support non-apriltag pipelines, so no point
  private PhotonCamera camera;
  // instead simulation can choose between maple sim coral or external photonvision client
  private CoralDetectionSimulationMode simulationMode = CoralDetectionSimulationMode.MAPLE_SIM;

  public enum CoralDetectionSimulationMode {
    PHOTONVISION,
    MAPLE_SIM
  }

  public static class DetectedCoral {
    public Pose3d fieldPose;
    public double timestamp;

    public DetectedCoral(Pose3d fieldPose, double timestamp) {
      this.fieldPose = fieldPose;
      this.timestamp = timestamp;
    }
  }

  record PhotonTrackedTargetTimestamped(PhotonTrackedTarget target, double timestamp) {}

  private List<DetectedCoral> detectedCorals = new ArrayList<>();

  public CoralDetection() {
    camera = new PhotonCamera(Constants.PhotonVision.PHOTON_CORAL_INTAKE);
  }

  @Override
  public void periodicManaged() {
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

      // Update coral

      var projected = project2dPoses(targets);
      for (int i = 0; i < projected.size(); i++) {
        var robot3d =
            new Pose3d(
                RobotContainer.poseSensorFusion.getEstimatedPositionAt(targets.get(i).timestamp));
        var camera3d =
            robot3d.transformBy(Constants.PhotonVision.groundIntakeTransformRobotToCamera);

        var projectedField = camera3d.transformBy(projected.get(i));
        boolean isNew = true;

        for (var coral : detectedCorals) {
          if (projectedField.getTranslation().getDistance(coral.fieldPose.getTranslation())
              < Constants.PhotonVision.CORAL_ID_DISTANCE.in(Meters)) {
            isNew = false;
            if (targets.get(i).timestamp > coral.timestamp) {
              coral.fieldPose = projectedField;
              coral.timestamp = targets.get(i).timestamp;
              break;
            }
          }
        }

        if (isNew) {
          detectedCorals.add(new DetectedCoral(projectedField, targets.get(i).timestamp));
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

  @AutoLogLevel(level = Level.DebugReal)
  public Pose3d[] getCorals() {
    if (Constants.RobotState.getMode() == Mode.REAL
        || simulationMode == CoralDetectionSimulationMode.PHOTONVISION) {
      Pose3d[] corals = new Pose3d[detectedCorals.size()];
      for (int i = 0; i < detectedCorals.size(); i++) {
        DetectedCoral coral = detectedCorals.get(i);
        corals[i] = coral.fieldPose;
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
