package frc.robot.control;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class JoystickXbox extends AbstractControl {

    private Joystick joystick;
    private XboxController xbox_controller;

    public JoystickXbox(int joystickPort, int xboxPort) {
		joystick = new Joystick(joystickPort);
		xbox_controller = new XboxController(xboxPort);
	}

    @Override
    public DriveCommandData getDriveCommandData() {
        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                getXY().getFirst() * getDirectionalSpeedLevel(),
                getXY().getSecond() * getDirectionalSpeedLevel(),
                getSpin() * getSpinSpeedLevel(),
                true);
        // Returns
        return driveCommandData;
    }

    public Pair<Double,Double> getXY() {
        double X = SimpleMath.ApplyThresholdAndSensitivity(joystick.getX(), Constants.Control.JOYSTICK_X_THRESHOLD, Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
        double Y = SimpleMath.ApplyThresholdAndSensitivity(joystick.getY(), Constants.Control.JOYSTICK_Y_THRESHOLD, Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
        return super.OrientXY(new Pair<Double,Double>(X, Y));
    }

    public Double getSpin() {
        // Gets raw twist value
        return SimpleMath.ApplyThresholdAndSensitivity(-SimpleMath.Remap(joystick.getTwist(), -1.0, 1.0, -1.0, 1.0), Constants.Control.JOYSTICK_SPIN_THRESHOLD, Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public Double getDirectionalSpeedLevel() {
		// Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
		return SimpleMath.Remap(
            joystick.getRawAxis(3), 
            1, 
            -1, 
            Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
			Constants.Control.DIRECTIONAL_SPEED_METER_HIGH
        );
	}

	public Double getSpinSpeedLevel() {
		// Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
		return SimpleMath.Remap(
            joystick.getRawAxis(3), 
            1, 
            -1, 
            Constants.Control.SPIN_SPEED_METER_LOW,
			Constants.Control.SPIN_SPEED_METER_HIGH
        );
	}

    @Override
    public Boolean getPoseReset() {
        return joystick.getRawButtonPressed(2);
    }

    @Override
    public Boolean getKillAuto() {
        return xbox_controller.getRawButton(8);
    }

    @Override
    public Boolean getAcquire() {
		return xbox_controller.getLeftTriggerAxis() > 0.3 || joystick.getRawButton(3); // aka the left trigger axis
	}

	@Override
	public Boolean getReverse() {
		return xbox_controller.getLeftBumper() || joystick.getRawButton(5); // aka the left trigger button
	}

	@Override
	public Boolean getShootSpeaker() {
		return xbox_controller.getRightTriggerAxis() > 0.3 || joystick.getRawButton(4); // aka the right trigger axis
	}

	@Override
	public Boolean getShootAmp() {
		return xbox_controller.getRightBumper() || joystick.getRawButton(6); // aka the right trigger button
	}

    @Override
    public Boolean getManualShootSpeaker() {
		return xbox_controller.getAButton();
	}

    @Override
	public Boolean getManualShootAmp() {
		return xbox_controller.getBButton();
	}

    @Override
	public Boolean getManualAcquisition() {
		return xbox_controller.getXButton();
	}

    @Override
	public Boolean getManualCrashbar() {
		return xbox_controller.getYButton();
	}

    @Override
    public Boolean getManualClimbers() {
        return xbox_controller.getRawButton(7);
    }

    @Override
    public Boolean getTeleAmp() {
        return joystick.getRawButton(3);
    }

    @Override
    public Boolean getTeleSpeaker() {
        return joystick.getRawButton(5);
    }

    @Override
    public Boolean getTeleChain() {
        return joystick.getRawButton(4);
    }
}