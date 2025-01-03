// Imports
package frc.robot.utils.drivemodes;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.ShuffleboardField;

public class TabletDrive {

    /**
     * Finds how fast the robot should be moving given tablet pressure
     * 
     * @return
     *         tablet pressure in m/s
     */
    private static double speedFromPressure(double tablet_pressure) {

        double PRESSURE_THRESHOLD = 0.2;
        double MIN_SPEED = 0.2;
        double STEEPNESS = 2.6; // Linear = 1, <1 = faster scaling, >1 = slower scaling

        if (tablet_pressure < PRESSURE_THRESHOLD) {
            return 0;
        }

        else {
            double coeff_1 = Math.pow((tablet_pressure - PRESSURE_THRESHOLD) / (1 - PRESSURE_THRESHOLD), STEEPNESS);
            double coeff_2 = 1 - MIN_SPEED;
            double final_speed = coeff_1 * coeff_2 + MIN_SPEED;
            return final_speed;
        }
    }

    // TODO: find a way to put the constants here in Constants.java
    /**
     * runs calculations for auto-orient
     * 
     * @return
     *         DriveCommandData object with drive directions
     */
    public static DriveCommandData calculate(DoubleControl _controls, double spin, Pose2d swerve_position) {

        // Puts raw tablet data on Smartdashboard
        SmartDashboard.putNumber("pressure", _controls.getTabletPressure());
        SmartDashboard.putNumber("tablet x", _controls.getTabletX());
        SmartDashboard.putNumber("tablet y", _controls.getTabletY());

        // Gets target speed, x, and y
        double speed = speedFromPressure(_controls.getTabletPressure());
        double target_x = _controls.getTabletX() * Constants.FieldConstants.FIELD_X_DIMENSION;
        double target_y = _controls.getTabletY() * Constants.FieldConstants.FIELD_Y_DIMENSION;
        SmartDashboard.putNumber("speed", speed);

        // Adds target pose
        ShuffleboardField.setTabletPos(target_x, target_y);

        // Calculates difference between target and current x and y
        double x_diff = target_x - swerve_position.getX();
        double y_diff = target_y - swerve_position.getY();

        // Calculates magnitude
        double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);
        double CLAMP_DISTANCE = 0.5;
        double clamped_magnitude = Math.min(1, magnitude / CLAMP_DISTANCE);

        // Calculates x and y speeds from magnitude and diff
        double SPEED_SCALING_FACTOR = 0.3;
        double x_speed = x_diff / magnitude * speed * SPEED_SCALING_FACTOR * clamped_magnitude;
        double y_speed = y_diff / magnitude * speed * SPEED_SCALING_FACTOR * clamped_magnitude;

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                x_speed, // x_speed,
                y_speed, // y_speed,
                spin,
                true);

        // Returns
        return driveCommandData;
    }
}
