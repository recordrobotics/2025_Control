package frc.robot.control;

public class ChildProtectiveServices extends JoystickXbox {
    
    public ChildProtectiveServices(int joystickPort, int xboxPort) {
		super(joystickPort, xboxPort);
	}

    @Override
    public Double getDirectionalSpeedLevel() {
        return super.getDirectionalSpeedLevel() * 0.4;
    }

    @Override
    public Double getSpinSpeedLevel() {
        return super.getDirectionalSpeedLevel() * 0.8;
    }
}
