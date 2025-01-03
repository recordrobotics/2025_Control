package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

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
}
