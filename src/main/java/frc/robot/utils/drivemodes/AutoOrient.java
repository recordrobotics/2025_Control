package frc.robot.utils.drivemodes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriverStationUtils;

public class AutoOrient {

    // Init variables
    private static PIDController anglePID = new PIDController(0.4, 0, 0);

    public AutoOrient() {
        // Enables continious input for PID
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Calculates the correct angle to move the robot to in order to face the target
     * position
     * 
     * @param robotPosition
     * @param targetPosition
     * @return
     */
    public static Rotation2d rotationFacingTarget(Translation2d robotPosition, Translation2d targetPosition) {
        return new Rotation2d(
                Math.atan2(robotPosition.getY() - targetPosition.getY(), robotPosition.getX() - targetPosition.getX()));
    }

    public static boolean shouldUpdateAngle(Translation2d robotPosition, Translation2d targetPosition) {
        return robotPosition.getDistance(targetPosition) > 0.05; // TODO: just a placeholder value, change later
    }

    /**
     * @return
     * Whether or not auto-orient should be running
     */
    public boolean shouldExecute(DoubleControl _controls) {
        return _controls.getAutoOrientSpeaker() || _controls.getAutoOrientAmp() && !_controls.getKillAuto();
    }

    /**
     * runs calculations for auto-orient
     * @return
     * DriveCommandData object with drive directions
     */
    public double calculate(DoubleControl _controls, Pose2d swerve_position) {

        // Calculates target position
        Translation2d targetPos = new Translation2d(0,0);
        if (_controls.getAutoOrientAmp()) {
            targetPos = DriverStationUtils.getCurrentAlliance() == Alliance.Red
                ? Constants.FieldConstants.TEAM_RED_AMP
                : Constants.FieldConstants.TEAM_BLUE_AMP;
        } else if (_controls.getAutoOrientSpeaker()) {
            targetPos = DriverStationUtils.getCurrentAlliance() == Alliance.Red
                ? Constants.FieldConstants.TEAM_RED_SPEAKER
                : Constants.FieldConstants.TEAM_BLUE_SPEAKER;
        }

        // Calculates spin (I don't know how  it works, TODO: get vlad to explain to me)
        double spin;
        if (shouldUpdateAngle(swerve_position.getTranslation(), targetPos)) {
        spin = Math.max(-0.5, Math.min(0.5, anglePID.calculate(swerve_position.getRotation().getRadians(),
            AutoOrient.rotationFacingTarget(swerve_position.getTranslation(), targetPos).getRadians())));
        } else {
        spin = 0;
        }
        
        // Returns spin
        return spin;
    }
}
