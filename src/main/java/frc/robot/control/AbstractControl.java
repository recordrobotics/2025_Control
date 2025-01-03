package frc.robot.control;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriverStationUtils;
import edu.wpi.first.math.Pair;


public abstract class AbstractControl {
    // Movement
    public abstract DriveCommandData getDriveCommandData();

    // Misc
    public abstract Boolean getPoseReset();
    public abstract Boolean getKillAuto();

    // Smart notes routines
    public abstract Boolean getAcquire();
    public abstract Boolean getReverse();
    public abstract Boolean getShootSpeaker();
    public abstract Boolean getShootAmp();

    // Manual
    public abstract Boolean getManualShootSpeaker();
    public abstract Boolean getManualShootAmp();
    public abstract Boolean getManualAcquisition();
    public abstract Boolean getManualCrashbar();
    public abstract Boolean getManualClimbers();

    // Tele scoring
    public abstract Boolean getTeleAmp();
    public abstract Boolean getTeleSpeaker();
    public abstract Boolean getTeleChain();

    // Orient XY
    public static Pair<Double,Double>OrientXY(Pair<Double,Double> input) {
        double inputX = input.getFirst();
        double inputY = input.getSecond();

		switch (ShuffleboardUI.Overview.getDriverOrientation()) {
			case XAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
                    return new Pair<Double,Double>(-inputY, -inputX);
				else
					return new Pair<Double,Double>(inputY, inputX);
			case YAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					return new Pair<Double,Double>(inputX, -inputY);
				else
					return new Pair<Double,Double>(-inputX, inputY);
			default:
				return new Pair<Double,Double>(0.0, 0.0);
		}
    }

    // Orient Angle
    public static Rotation2d OrientAngle(Rotation2d angle) {
        switch (ShuffleboardUI.Overview.getDriverOrientation()) {
            case XAxisTowardsTrigger:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
                    return new Rotation2d(angle.getRadians() - Math.PI / 2);
                else
                    return new Rotation2d(angle.getRadians() + Math.PI / 2);
            case YAxisTowardsTrigger:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
                    return angle;
                else
                    return new Rotation2d(angle.getRadians() + Math.PI);
            default:
                return angle;
        }
    }
}