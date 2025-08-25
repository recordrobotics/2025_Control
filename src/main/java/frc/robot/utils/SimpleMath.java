package frc.robot.utils;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Random;

public final class SimpleMath {
    private SimpleMath() {}

    public static final double PI2 = Math.PI * 2;
    public static final double SQRT2 = Math.sqrt(2);
    public static final double SECONDS_PER_MINUTE = 60;

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
    public static double remap(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
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
    public static double remap(double value01, double min, double max) {
        return remap(MathUtil.clamp(value01, 0, 1), 0, 1, min, max);
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
    public static double applyThresholdAndSensitivity(double input, double threshold, double sensitivity) {
        // How much the input is above the threshold (absolute value)
        double subtractThreshold = Math.max(0, Math.abs(input) - threshold);
        // What proportion (threshold to value) is of (threshold to 1)
        double proportion = subtractThreshold / (1 - threshold);
        // Multiplies by spin sensitivity and returns
        return Math.signum(input) * proportion * sensitivity;
    }

    public static boolean isPoseInField(Pose2d pose) {
        return pose.getX() >= 0
                && pose.getY() >= 0
                && pose.getX() <= FlippingUtil.fieldSizeX
                && pose.getY() <= FlippingUtil.fieldSizeY;
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

    public static double slewRateLimitLinear(double current, double next, double dt, double maxVelocity) {
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

    @SuppressWarnings("java:S109")
    public static Translation2d povToVector(int pov) {
        return switch (pov) {
            case 0 -> new Translation2d(0, 1);
            case 45 -> new Translation2d(1, 1);
            case 90 -> new Translation2d(1, 0);
            case 135 -> new Translation2d(1, -1);
            case 180 -> new Translation2d(0, -1);
            case 225 -> new Translation2d(-1, -1);
            case 270 -> new Translation2d(-1, 0);
            case 315 -> new Translation2d(-1, 1);
            default -> new Translation2d(0, 0);
        };
    }

    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) <= tolerance;
    }

    private static final Random rand = new Random();

    public static Pose2d poseNoise(Pose2d pose, double stdDev, double stdDevRot) {
        double x = pose.getX() + rand.nextGaussian(0, stdDev);
        double y = pose.getY() + rand.nextGaussian(0, stdDev);
        double rot = pose.getRotation().getRadians() + rand.nextGaussian(0, stdDevRot);

        return new Pose2d(new Translation2d(x, y), new Rotation2d(rot));
    }

    /**
     * Returns the closest unbounded axis position corresponding to the target angle.
     *
     * @param currentPos Current unbounded axis position (radians)
     * @param targetAngle Target angle (radians, in [-PI, PI])
     * @return Closest unbounded position to the target angle
     */
    public static double closestTarget(double currentPos, double targetAngle) {
        // Compute the nearest multiple of 2pi
        double k = Math.round((currentPos - targetAngle) / PI2);
        // Return the unbounded target position
        return targetAngle + k * PI2;
    }

    /**
     *  Normalize an angle to [-PI, PI] (for use with {@link #closestTarget})
     */
    public static double normalizeAngle(double angle) {
        angle = ((angle + Math.PI) % PI2 + PI2) % PI2 - Math.PI;
        return angle;
    }

    @SuppressWarnings("java:S1244") // comparing signum is fine because it returns EXACT values
    public static boolean signEq(double a, double b) {
        return Math.signum(a) == Math.signum(b);
    }

    public static double average(double... values) {
        if (values.length == 0) {
            throw new IllegalArgumentException("Cannot compute average of zero values");
        }
        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.length;
    }

    @SuppressWarnings("java:S109")
    public static double average4(double a, double b, double c, double d) {
        return (a + b + c + d) / 4.0;
    }
}
