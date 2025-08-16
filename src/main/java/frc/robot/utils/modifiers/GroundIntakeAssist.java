package frc.robot.utils.modifiers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeAssist implements IDrivetrainControlModifier {

    private boolean enabled = true;

    public enum FailReason {
        NONE,
        NO_OR_UNREACHABLE_CORAL,
        NOT_IN_GROUND_MODE,
        NOT_DRIVING,
        TOO_MUCH_ANGLE_ERROR,
        DRIVER_MOVING_IN_OPPOSITE_DIRECTION,
        DRIVER_FIGHTING_ASSIST,
        DRIVER_ROTATE_IN_OPPOSITE_DIRECTION,
        DRIVER_ROTATE_FIGHTING_ASSIST
    }

    @Override
    public boolean apply(DrivetrainControl control) {

        // Ground assist requires coral intake to be in ground mode
        if (RobotContainer.coralIntake.getState() != CoralIntakeState.GROUND) {
            Logger.recordOutput("GroundIntakeAssist/Corals", new Pose3d[0]);
            Logger.recordOutput("GroundIntakeAssist/TargetCoral", new Pose3d[0]);
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.NOT_IN_GROUND_MODE);
            return false;
        }

        Pose2d robotPose = RobotContainer.poseSensorFusion
                .getEstimatedPosition()
                .transformBy(new Transform2d(Constants.CoralIntake.INTAKE_X_OFFSET, Meters.zero(), Rotation2d.kZero));

        Translation2d driverVector = DrivetrainControl.robotToField(
                        control.getTargetVelocity(), robotPose.getRotation())
                .getTranslation();

        double driverSpeed = driverVector.getNorm();
        driverVector.div(driverSpeed);

        // If the driver is not moving, don't assist
        if (driverSpeed < 0.1) {
            Logger.recordOutput("GroundIntakeAssist/Corals", new Pose3d[0]);
            Logger.recordOutput("GroundIntakeAssist/TargetCoral", new Pose3d[0]);
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.NOT_DRIVING);
            return false;
        }

        // Find all coral in direction of driver velocity
        List<Pose3d> corals = new ArrayList<>();
        Pose3d closestCoral = null;
        double closestCoralDistance = Double.MAX_VALUE;
        for (Pose3d coral : RobotContainer.coralDetection.getCorals()) {
            Translation2d robotToCoral =
                    coral.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
            robotToCoral = robotToCoral.div(robotToCoral.getNorm());

            double dot = robotToCoral.getX() * driverVector.getX() + robotToCoral.getY() * driverVector.getY();

            if (dot > 0.85) {
                corals.add(coral);

                double distance = robotPose
                        .getTranslation()
                        .getDistance(coral.getTranslation().toTranslation2d());
                if (distance <= Constants.Assits.GROUND_ASSIST_MAX_CORAL_DISTANCE.in(Meters)
                        && distance < closestCoralDistance) {
                    closestCoralDistance = distance;
                    closestCoral = coral;
                }
            }
        }

        Logger.recordOutput("GroundIntakeAssist/Corals", corals.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "GroundIntakeAssist/TargetCoral", closestCoral == null ? new Pose3d[0] : new Pose3d[] {closestCoral});

        if (closestCoral == null) {
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.NO_OR_UNREACHABLE_CORAL);
            return false;
        }

        // Find the angle to the closest coral
        Translation2d robotToCoral =
                closestCoral.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
        double angle =
                Math.atan2(robotToCoral.getY(), robotToCoral.getX()) - Math.PI / 2; // face ground intake towards coral

        Logger.recordOutput(
                "GroundIntakeAssist/TargetPose", new Pose2d(robotPose.getTranslation(), new Rotation2d(angle)));

        double angleDiff = angle - robotPose.getRotation().getRadians();

        if (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        if (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

        // If the angle is too large, don't assist
        if (Math.abs(angleDiff) > Constants.Assits.GROUND_ASSIST_MAX_ANGLE_ERROR.in(Radians)) {
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.TOO_MUCH_ANGLE_ERROR);
            return false;
        }

        Rotation2d targetAngularVelocity = new Rotation2d(angleDiff * Constants.Assits.GROUND_ASSIST_ROTATION_P);

        Translation2d targetVelocity = DrivetrainControl.fieldToRobot(robotToCoral, robotPose.getRotation())
                .times(Constants.Assits.GROUND_ASSIST_TRANSLATION_P);

        double driverX = control.getTargetVelocity().getX();
        double driverRot = control.getTargetVelocity().getRotation().getRadians();

        // If the driver is moving in the opposite direction of the target velocity, don't assist
        if (Math.signum(driverX) != 0
                && Math.signum(driverX) != Math.signum(targetVelocity.getX())
                && Math.abs(driverX - targetVelocity.getX()) > 2.5) {
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.DRIVER_MOVING_IN_OPPOSITE_DIRECTION);
            return false;
        }

        // If driver is fighting assist, stop
        if (Math.abs(driverX - targetVelocity.getX()) > 3.0) {
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.DRIVER_FIGHTING_ASSIST);
            return false;
        }

        // If the driver is moving in the opposite direction of the target velocity, don't assist
        if (Math.signum(driverRot) != 0
                && Math.signum(driverRot) != Math.signum(targetAngularVelocity.getRadians())
                && Math.abs(driverRot - targetAngularVelocity.getRadians()) > Units.degreesToRadians(40)) {
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.DRIVER_ROTATE_IN_OPPOSITE_DIRECTION);
            return false;
        }

        // If driver is fighting assist, stop
        if (Math.abs(driverRot - targetAngularVelocity.getRadians()) > Units.degreesToRadians(70)) {
            Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.DRIVER_ROTATE_FIGHTING_ASSIST);
            return false;
        }

        control.applyWeightedVelocity(
                new Transform2d(
                        targetVelocity.getX(), control.getTargetVelocity().getY(), targetAngularVelocity),
                MathUtil.interpolate(0.6, 1.0, MathUtil.inverseInterpolate(0.1, 0.5, driverSpeed)));

        Logger.recordOutput("GroundIntakeAssist/FailReason", FailReason.NONE);

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
