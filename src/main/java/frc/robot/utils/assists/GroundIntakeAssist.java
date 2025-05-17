package frc.robot.utils.assists;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeAssist implements IAssist {

  private boolean enabled = true;

  @Override
  public boolean apply(DrivetrainControl control) {

    // Ground assist requires coral intake to be in ground mode
    if (RobotContainer.coralIntake.getState() != CoralIntakeState.GROUND) {
      Logger.recordOutput("GroundIntakeAssist/Corals", new Pose3d[0]);
      Logger.recordOutput("GroundIntakeAssist/TargetCoral", new Pose3d[0]);
      return false;
    }

    var robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
    var driverVector =
        DrivetrainControl.robotToField(control.getDriverVelocity(), robotPose.getRotation())
            .getTranslation();

    double driverSpeed = driverVector.getNorm();
    driverVector.div(driverSpeed);

    // If the driver is not moving, don't assist
    if (driverSpeed < 0.1) {
      Logger.recordOutput("GroundIntakeAssist/Corals", new Pose3d[0]);
      Logger.recordOutput("GroundIntakeAssist/TargetCoral", new Pose3d[0]);
      return false;
    }

    // Find all coral in direction of driver velocity
    List<Pose3d> corals = new ArrayList<>();
    Pose3d closestCoral = null;
    double closestCoralDistance = Double.MAX_VALUE;
    for (Pose3d coral : RobotContainer.coralDetection.getCorals()) {
      var robotToCoral = coral.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
      robotToCoral = robotToCoral.div(robotToCoral.getNorm());

      double dot =
          robotToCoral.getX() * driverVector.getX() + robotToCoral.getY() * driverVector.getY();

      if (dot > 0.85) {
        corals.add(coral);

        double distance =
            robotPose.getTranslation().getDistance(coral.getTranslation().toTranslation2d());
        if (distance <= Constants.Assits.GROUND_ASSIST_MAX_CORAL_DISTANCE.in(Meters)
            && distance < closestCoralDistance) {
          closestCoralDistance = distance;
          closestCoral = coral;
        }
      }
    }

    Logger.recordOutput("GroundIntakeAssist/Corals", corals.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "GroundIntakeAssist/TargetCoral",
        closestCoral == null ? new Pose3d[0] : new Pose3d[] {closestCoral});

    if (closestCoral == null) return false;

    // Find the angle to the closest coral
    var robotToCoral =
        closestCoral.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
    double angle =
        Math.atan2(robotToCoral.getY(), robotToCoral.getX())
            - Math.PI / 2; // face ground intake towards coral

    Logger.recordOutput(
        "GroundIntakeAssist/TargetPose",
        new Pose2d(robotPose.getTranslation(), new Rotation2d(angle)));

    double angleDiff = angle - robotPose.getRotation().getRadians();

    if (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
    if (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

    // If the angle is too large, don't assist
    if (Math.abs(angleDiff) > Constants.Assits.GROUND_ASSIST_MAX_ANGLE_ERROR.in(Radians)) {
      return false;
    }

    Rotation2d targetAngularVelocity =
        new Rotation2d(angleDiff * Constants.Assits.GROUND_ASSIST_ROTATION_P);

    var targetVelocity =
        DrivetrainControl.fieldToRobot(robotToCoral, robotPose.getRotation())
            .times(Constants.Assits.GROUND_ASSIST_TRANSLATION_P);

    double driverX = control.getDriverVelocity().getX();
    double driverRot = control.getDriverVelocity().getRotation().getRadians();

    // If the driver is moving in the opposite direction of the target velocity, don't assist
    if (Math.signum(driverX) != 0
        && Math.signum(driverX) != Math.signum(targetVelocity.getX())
        && Math.abs(driverX - targetVelocity.getX()) > 0.5) {
      return false;
    }

    // If driver is fighting assist, stop
    if (Math.abs(driverX - targetVelocity.getX()) > 1.5) {
      return false;
    }

    // If the driver is moving in the opposite direction of the target velocity, don't assist
    if (Math.signum(driverRot) != 0
        && Math.signum(driverRot) != Math.signum(targetAngularVelocity.getRadians())
        && Math.abs(driverRot - targetAngularVelocity.getRadians()) > Units.degreesToRadians(20)) {
      return false;
    }

    // If driver is fighting assist, stop
    if (Math.abs(driverRot - targetAngularVelocity.getRadians()) > Units.degreesToRadians(60)) {
      return false;
    }

    control.applyWeightedVelocity(
        new Transform2d(
            targetVelocity.getX(), control.getTargetVelocity().getY(), targetAngularVelocity),
        MathUtil.interpolate(0.4, 1.0, MathUtil.inverseInterpolate(0.1, 0.5, driverSpeed)));

    return true;
  }

  @Override
  public boolean isEnabled() {
    return enabled;
  }

  @Override
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }
}
