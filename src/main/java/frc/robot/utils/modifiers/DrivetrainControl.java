package frc.robot.utils.modifiers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainControl {

    private Transform2d driverVelocity;
    private Transform2d driverAcceleration;
    private Transform2d driverJerk;

    private Transform2d targetVelocity;

    private DrivetrainControl(Transform2d driverVelocity, Transform2d driverAcceleration, Transform2d driverJerk) {
        this.driverVelocity = driverVelocity;
        this.driverAcceleration = driverAcceleration;
        this.driverJerk = driverJerk;

        this.targetVelocity = driverVelocity;
    }

    public static Transform2d fieldToRobot(Transform2d fieldRelative, Rotation2d robotAngle) {
        // CW rotation into chassis frame
        return new Transform2d(
                fieldRelative.getTranslation().rotateBy(robotAngle.unaryMinus()), fieldRelative.getRotation());
    }

    public static Transform2d robotToField(Transform2d robotRelative, Rotation2d robotAngle) {
        // CCW rotation into field frame
        return new Transform2d(robotRelative.getTranslation().rotateBy(robotAngle), robotRelative.getRotation());
    }

    public static Translation2d fieldToRobot(Translation2d fieldRelative, Rotation2d robotAngle) {
        // CW rotation into chassis frame
        return fieldRelative.rotateBy(robotAngle.unaryMinus());
    }

    public static Translation2d robotToField(Translation2d robotRelative, Rotation2d robotAngle) {
        // CCW rotation into field frame
        return robotRelative.rotateBy(robotAngle);
    }

    public static DrivetrainControl createRobotRelative(
            Transform2d driverVelocity, Transform2d driverAcceleration, Transform2d driverJerk) {
        return new DrivetrainControl(driverVelocity, driverAcceleration, driverJerk);
    }

    public static DrivetrainControl createFieldRelative(
            Transform2d driverVelocity, Transform2d driverAcceleration, Transform2d driverJerk, Rotation2d robotAngle) {

        return new DrivetrainControl(
                fieldToRobot(driverVelocity, robotAngle),
                fieldToRobot(driverAcceleration, robotAngle),
                fieldToRobot(driverJerk, robotAngle));
    }

    /**
     * @deprecated Warning: the driver velocity does not change with applied modifiers, making
     *     stacking modifiers challenging. Only use this if you know what you're doing
     *     {@code @SuppressWarnings("deprecation")}
     */
    @Deprecated
    public Transform2d getDriverVelocity() {
        return driverVelocity;
    }

    public Transform2d getDriverAcceleration() {
        return driverAcceleration;
    }

    public Transform2d getDriverJerk() {
        return driverJerk;
    }

    public Transform2d getTargetVelocity() {
        return targetVelocity;
    }

    public ChassisSpeeds toChassisSpeeds() {
        return new ChassisSpeeds(
                targetVelocity.getTranslation().getX(),
                targetVelocity.getTranslation().getY(),
                targetVelocity.getRotation().getRadians());
    }

    /**
     * Sets target velocity to the given value interpolated using weight.
     *
     * @param velocity The velocity to apply
     * @param weight The weight to interpolate with [0, 1]
     */
    public void applyWeightedVelocity(Transform2d velocity, double weight) {
        targetVelocity = new Transform2d(
                targetVelocity.getTranslation().interpolate(velocity.getTranslation(), weight),
                targetVelocity.getRotation().interpolate(velocity.getRotation(), weight));
    }
}
