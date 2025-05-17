package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class SimpleMath {

  /**
   * Remaps a value between a range to a different range
   *
   * @param value Input value
   * @param inputMin Min range of input value
   * @param inputMax Max range of input value
   * @param outputMin Min range of output value
   * @param outputMax Max range of output value
   * @return Value in range of output range
   * @apiNote THIS DOES NOT CLAMP THE OUTPUT! If input is outside the input range, the output will
   *     also be outside the output range
   */
  public static double Remap(
      double value, double inputMin, double inputMax, double outputMin, double outputMax) {
    return (value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin;
  }

  /**
   * Remaps a value between 0 and 1 to a range, while clamping inside the range
   *
   * @param value01 Input value BETWEEN 0 and 1
   * @param min Min range of output value
   * @param max Max range of output value
   * @return Value in range of output range (ALWAYS INSIDE, CLAMPED)
   */
  public static double Remap(double value01, double min, double max) {
    return Remap(MathUtil.clamp(value01, 0, 1), 0, 1, min, max);
  }

  /**
   * Takes an input value between -1 and 1 and scales it to the proportion to which it's absolute
   * value is between a minimum threshold and 1 (Function returns 0 if input < threshold) Then
   * multiplies by sensitivity and returns
   *
   * @param input
   * @param threshold
   * @param sensitivity
   * @return
   */
  public static double ApplyThresholdAndSensitivity(
      double input, double threshold, double sensitivity) {
    // How much the input is above the threshold (absolute value)
    double subtract_threshold = Math.max(0, Math.abs(input) - threshold);
    // What proportion (threshold to value) is of (threshold to 1)
    double proportion = subtract_threshold / (1 - threshold);
    // Multiplies by spin sensitivity and returns
    return Math.signum(input) * proportion * sensitivity;
  }

  public static Translation2d MirrorLocation(Translation2d location) {
    double mirrored_x = Constants.FieldConstants.FIELD_X_DIMENSION - location.getX();
    double mirrored_y = location.getY();
    return new Translation2d(mirrored_x, mirrored_y);
  }

  public static boolean isPoseInField(Pose2d pose) {
    return pose.getX() >= 0
        && pose.getY() >= 0
        && pose.getX() <= Constants.FieldConstants.FIELD_X_DIMENSION
        && pose.getY() <= Constants.FieldConstants.FIELD_Y_DIMENSION;
  }

  /**
   * Converts a Translation3d to a Rotation3d
   *
   * <p>This function takes a Translation3d and returns a Rotation3d object with the yaw and pitch
   * calculated from the translation. The roll is set to 0 as it's not required.
   *
   * <p>The yaw is calculated as the angle from the X axis to the point (x, y) in the XY plane. The
   * pitch is calculated as the angle from the XY plane to the point (x, y, z) in 3D space.
   *
   * @param translation The Translation3d to convert
   * @return A Rotation3d with the calculated yaw and pitch
   */
  public static Rotation3d translationToRotation(Translation3d translation) {
    double x = translation.getX();
    double y = translation.getY();
    double z = translation.getZ();

    // Calculate yaw (rotation around Z axis)
    double yaw = Math.atan2(y, x);

    // Calculate pitch (rotation around Y axis)
    double horizontalDistance = Math.sqrt(x * x + y * y);
    double pitch = Math.atan2(z, horizontalDistance);

    // Roll is set to 0 as it's not required
    return new Rotation3d(0.0, pitch, yaw);
  }

  public static double slewRateLimitLinear(
      double current, double next, double dt, double maxVelocity) {
    if (maxVelocity < 0) {
      Exception e = new IllegalArgumentException();
      MathSharedStore.reportError(
          "maxVelocity must be a non-negative number, got " + maxVelocity, e.getStackTrace());
      return next;
    }
    double diff = next - current;
    double dist = Math.abs(diff);

    if (dist < 1e-9) {
      return next;
    }
    if (dist > maxVelocity * dt) {
      // Move maximum allowed amount in direction of the difference
      return current + (diff * (maxVelocity * dt / dist));
    }
    return next;
  }

  public static Translation2d povToVector(int pov) {
    switch (pov) {
      case 0:
        return new Translation2d(0, 1);
      case 45:
        return new Translation2d(1, 1);
      case 90:
        return new Translation2d(1, 0);
      case 135:
        return new Translation2d(1, -1);
      case 180:
        return new Translation2d(0, -1);
      case 225:
        return new Translation2d(-1, -1);
      case 270:
        return new Translation2d(-1, 0);
      case 315:
        return new Translation2d(-1, 1);
      default:
        return new Translation2d(0, 0);
    }
  }

  public static boolean isWithinTolerance(double value, double target, double tolerance) {
    return Math.abs(value - target) <= tolerance;
  }
}
