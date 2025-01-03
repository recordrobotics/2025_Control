package frc.robot.control;

/**
 * Specifies all control inputs needed for the robot
 */
public interface IControlInput {

    double getX();

    double getY();

    double getSpin();

    boolean toggleClimbers();

    boolean spinFlywheel();

    boolean getAcquisition();

    boolean getResetPressed();

    double getSpeedLevel();
}
