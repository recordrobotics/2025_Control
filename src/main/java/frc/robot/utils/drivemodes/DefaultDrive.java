// Imports
package frc.robot.utils.drivemodes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

//
public class DefaultDrive {

    /**
     * runs calculations for auto-orient
     * 
     * @return
     *         DriveCommandData object with drive directions
     */
    public static DriveCommandData calculate(DoubleControl _controls, double spin, Pose2d swerve_position,
            boolean field_relative) {

        // Gets speed level from controller
        double speedLevel = _controls.getSpeedLevel();
        double spinSpeedLevel = SimpleMath.Remap(_controls.getSpeedLevelNormalized(), 1.4, 7.0);

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                _controls.getX() * speedLevel,
                _controls.getY() * speedLevel,
                spin * spinSpeedLevel,
                field_relative);

        // Returns
        return driveCommandData;
    }
}
