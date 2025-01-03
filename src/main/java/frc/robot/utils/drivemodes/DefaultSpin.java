package frc.robot.utils.drivemodes;
import frc.robot.control.DoubleControl;

public class DefaultSpin {

    /**
     * @return
     * spin given by the joystick
     */
    public static double calculate(DoubleControl _controls) {
        return _controls.getSpin(); 
    }

}
