package frc.robot.control;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    DriveCommandData driveCommandData =
        new DriveCommandData(
            getXY().getFirst() * getDirectionalSpeedLevel(),
            getXY().getSecond() * getDirectionalSpeedLevel(),
            getSpin() * getSpinSpeedLevel(),
            true);
    // Returns
    return driveCommandData;
  }

  public Boolean getAutoAlign() {
    return joystick.getRawButton(7)
        || joystick.getRawButton(8)
        || joystick.getRawButton(9)
        || joystick.getRawButton(10)
        || joystick.getRawButton(11)
        || joystick.getRawButton(12);
  }

  public Pair<Double, Double> getXY() {
    double X =
        SimpleMath.ApplyThresholdAndSensitivity(
            joystick.getX(),
            Constants.Control.JOYSTICK_X_THRESHOLD,
            Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
    double Y =
        SimpleMath.ApplyThresholdAndSensitivity(
            joystick.getY(),
            Constants.Control.JOYSTICK_Y_THRESHOLD,
            Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
    return super.OrientXY(new Pair<Double, Double>(X, Y));
  }

  public Double getSpin() {
    // Gets raw twist value
    return SimpleMath.ApplyThresholdAndSensitivity(
        -SimpleMath.Remap(joystick.getTwist(), -1.0, 1.0, -1.0, 1.0),
        Constants.Control.JOYSTICK_SPIN_THRESHOLD,
        Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
  }

  public Double getDirectionalSpeedLevel() {
    // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
    return SimpleMath.Remap(
        joystick.getRawAxis(3),
        1,
        -1,
        Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
        Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);
  }

  public Double getSpinSpeedLevel() {
    // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
    return SimpleMath.Remap(
        joystick.getRawAxis(3),
        1,
        -1,
        Constants.Control.SPIN_SPEED_METER_LOW,
        Constants.Control.SPIN_SPEED_METER_HIGH);
  }

  @Override
  public Boolean getPoseReset() {
    return joystick.getRawButtonPressed(3)
        || joystick.getRawButtonPressed(4)
        || joystick.getRawButtonPressed(5)
        || joystick.getRawButtonPressed(6);
  }

  @Override
  public Boolean getKill() {
    return xbox_controller.getRawButton(8);
  }

  @Override
  public void vibrate(double value) {
    xbox_controller.setRumble(RumbleType.kBothRumble, value);
  }

  @Override
  public Boolean getElevatorL1() {
    return xbox_controller.getAButton();
  }

  @Override
  public Boolean getElevatorL2() {
    return xbox_controller.getXButton();
  }

  @Override
  public Boolean getElevatorL3() {
    return xbox_controller.getBButton();
  }

  @Override
  public Boolean getElevatorL4() {
    return xbox_controller.getYButton();
  }

  @Override
  public Boolean getCoralShoot() {
    return xbox_controller.getPOV() == 270;
  }

  @Override
  public Boolean getCoralGroundIntake() {
    return xbox_controller.getLeftTriggerAxis() > 0.3;
  }

  @Override
  public Boolean getCoralSourceIntake() {
    return xbox_controller.getLeftBumperButton();
  }

  @Override
  public Boolean getElevatorAlgaeLow() {
    return xbox_controller.getRightTriggerAxis() > 0.3 && xbox_controller.getPOV() != 0;
  }

  @Override
  public Boolean getElevatorAlgaeHigh() {
    return xbox_controller.getRightTriggerAxis() > 0.3 && xbox_controller.getPOV() == 0;
  }

  @Override
  public Boolean getGroundAlgae() {
    return xbox_controller.getRightBumperButton();
  }

  @Override
  public Boolean getScoreAlgae() {
    return xbox_controller.getPOV() == 90;
  }

  @Override
  public Boolean getCoralIntakeScoreL1() {
    return xbox_controller.getPOV() == 180;
  }

  @Override
  public Boolean getClimb() {
    return xbox_controller.getRawButton(1);
  }

  @Override
  public LinearVelocity getManualElevatorVelocity() {
    double leftY = MathUtil.applyDeadband(-xbox_controller.getLeftY(), 0.1);
    return Centimeters.of(50).per(Seconds).times(leftY * Math.abs(leftY));
  }

  @Override
  public AngularVelocity getManualElevatorArmVelocity() {
    double rightY = MathUtil.applyDeadband(-xbox_controller.getRightY(), 0.1);
    return Degrees.of(180).per(Seconds).times(rightY * Math.abs(rightY));
  }
}
