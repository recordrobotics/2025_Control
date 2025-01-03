package frc.robot.utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class SimpleMath {
    
    /**
     * Remaps a value between a range to a different range
     * 
     * @param value     Input value
     * @param inputMin  Min range of input value
     * @param inputMax  Max range of input value
     * @param outputMin Min range of output value
     * @param outputMax Max range of output value
     * @return Value in range of output range
     * @apiNote THIS DOES NOT CLAMP THE OUTPUT! If input is outside the input range,
     *          the output will also be outside the output range
     */
    public static double Remap(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin;
    }

    /**
     * Remaps a value between 0 and 1 to a range, while clamping inside the range
     * 
     * @param value01 Input value BETWEEN 0 and 1
     * @param min     Min range of output value
     * @param max     Max range of output value
     * @return Value in range of output range (ALWAYS INSIDE, CLAMPED)
     */
    public static double Remap(double value01, double min, double max) {
        return Remap(MathUtil.clamp(value01, 0, 1), 0, 1, min, max);
    }

    /**
     * Takes an input value between -1 and 1 and scales it to 
     * the proportion to which it's absolute value is between a minimum threshold and 1 
     * (Function returns 0 if input < threshold)
     * Then multiplies by sensitivity and returns
     * @param input
     * @param threshold
     * @param sensitivity
     * @return
     */
    public static double ApplyThresholdAndSensitivity(double input, double threshold, double sensitivity) {
        // How much the input is above the threshold (absolute value)
		double subtract_threshold = Math.max(0, Math.abs(input) - threshold);
		// What proportion (threshold to value) is of (threshold to 1)
		double proportion = subtract_threshold / (1 - threshold);
		// Multiplies by spin sensitivity and returns
		return Math.signum(input) * proportion * sensitivity;
    }

    /**
     * //TODO: fix because I think the field dimension is wrong
     * @param location
     * @return mirrored translation2d of the translation2d you put in
     */
    public static Translation2d MirrorLocation(Translation2d location) {
        double mirrored_x = Constants.FieldConstants.FIELD_X_DIMENSION - location.getX();
        double mirrored_y = location.getY();
        return new Translation2d(mirrored_x, mirrored_y);
    }

}