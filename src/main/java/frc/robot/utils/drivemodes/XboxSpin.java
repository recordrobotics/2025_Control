package frc.robot.utils.drivemodes;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.DoubleControl;

public class XboxSpin {

    // Init variables
    private PIDController anglePID = new PIDController(0.4, 0, 0);

    public XboxSpin() {
        // Enables continious input for PID
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static boolean shouldUpdateAngle(Translation2d robotPosition, Translation2d targetPosition) {
        return robotPosition.getDistance(targetPosition) > 0.05; // TODO: just a placeholder value, change later
    }

    public boolean shouldExecute(DoubleControl _controls) {
        boolean is_over_threshold = _controls.getXboxSpinAngle().getSecond();
        return is_over_threshold; 
    }

    public double calculate(DoubleControl _controls, Pose2d swerve_position) {

        boolean is_over_threshold = _controls.getXboxSpinAngle().getSecond();

        if (is_over_threshold) {
            double xbox_angle = _controls.getXboxSpinAngle().getFirst();
            double robot_angle = swerve_position.getRotation().getRadians();

            double spin = Math.max(-0.5, Math.min(0.5, anglePID.calculate(robot_angle, xbox_angle)));
            
            // Returns spin
            return spin;
        }
        return 0;
    }

}