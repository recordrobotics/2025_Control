package frc.robot.control;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class JoystickXboxSimple extends AbstractControl {

  private Joystick joystick;
  private XboxController xbox_controller;

  public JoystickXboxSimple(int joystickPort, int xboxPort) {
    joystick = new Joystick(joystickPort);
    xbox_controller = new XboxController(xboxPort);
  }

  @Override
  public DriveCommandData getDriveCommandData() {
    // Gets information needed to drive

    var xy = getXY(!(getCoralIntakeRelativeDrive() || getElevatorRelativeDrive()));

    double x = xy.getFirst() * getDirectionalSpeedLevel();
    double y = xy.getSecond() * getDirectionalSpeedLevel();

    if (getCoralIntakeRelativeDrive()) {
      y = -y;
    } else if (getElevatorRelativeDrive()) {
      double temp = y;
      y = -x;
      x = -temp;
    }

    DriveCommandData driveCommandData =
        new DriveCommandData(
            x,
            y,
            getSpin() * getSpinSpeedLevel(),
            !(getElevatorRelativeDrive() || getCoralIntakeRelativeDrive()));
    // Returns
    return driveCommandData;
  }

  public Boolean getAutoAlign() {
    return joystick.getRawButton(9) || joystick.getRawButton(11);
  }

  public Boolean getAutoAlignNear() {
    return joystick.getRawButton(7) || joystick.getRawButton(12);
  }

  public Boolean getElevatorRelativeDrive() {
    return joystick.getRawButton(8) || joystick.getRawButton(2);
  }

  public Boolean getCoralIntakeRelativeDrive() {
    return joystick.getRawButton(10);
  }

  public Pair<Double, Double> getXY(boolean orient) {
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

    if (orient) return super.OrientXY(new Pair<Double, Double>(X, Y));
    else return new Pair<Double, Double>(X, Y);
  }

  public Double getSpin() {
    // Gets raw twist value
    return SimpleMath.ApplyThresholdAndSensitivity(
        -SimpleMath.Remap(joystick.getTwist(), -1.0, 1.0, -1.0, 1.0),
        Constants.Control.JOYSTICK_SPIN_THRESHOLD,
        Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
  }

  public Boolean getHalfSpeed() {
    return joystick.getRawButton(1);
  }

  public Double getDirectionalSpeedLevel() {
    // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
    double speed =
        SimpleMath.Remap(
            joystick.getRawAxis(3),
            1,
            -1,
            Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
            Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);

    if (getHalfSpeed()) {
      speed /= 3;
    }

    return speed;
  }

  public Double getSpinSpeedLevel() {
    // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
    double speed =
        SimpleMath.Remap(
            joystick.getRawAxis(3),
            1,
            -1,
            Constants.Control.SPIN_SPEED_METER_LOW,
            Constants.Control.SPIN_SPEED_METER_HIGH);

    if (getHalfSpeed()) {
      speed /= 2;
    }

    return speed;
  }

  @Override
  public Boolean getPoseReset() {
    return joystick.getRawButtonPressed(3) || joystick.getRawButtonPressed(5);
  }

  @Override
  public Boolean getLimelightReset() {
    return joystick.getRawButtonPressed(4) || joystick.getRawButtonPressed(6);
  }

  @Override
  public Boolean getKill() {
    return xbox_controller.getRawButton(8);
  }

  @Override
  public Boolean getClimbMode() {
    return joystick.getRawButton(12);
  }

  @Override
  public void vibrate(RumbleType type, double value) {
    xbox_controller.setRumble(type, value);
  }

  @Override
  public Boolean getAutoScore() {
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
  public AutoScoreDirection getAutoScoreDirection() {
    if (xbox_controller.getLeftX() < -0.5) {
      return AutoScoreDirection.Left;
    } else if (xbox_controller.getLeftX() > 0.5) {
      return AutoScoreDirection.Right;
    } else {
      return AutoScoreDirection.None;
    }
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
    return false;
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
  public Boolean getManualOverride() {
    return xbox_controller.getPOV() == 0;
  }

  @Override
  public Boolean getGroundAlgae() {
    return false;
  }

  @Override
  public Boolean getScoreAlgae() {
    return false;
  }

  @Override
  public Boolean getCoralIntakeScoreL1() {
    return xbox_controller.getPOV() == 180;
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

  @Override
  public Boolean getClimb() {
    return xbox_controller.getRawButton(7);
  }

  @Override
  public Boolean getCoralSourceIntakeAuto() {
    Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
    SourcePosition closestSource = SourcePosition.closestTo(robot);

    boolean nearSource =
        closestSource.getPose().getTranslation().getDistance(robot.getTranslation()) < 2.3
            && Math.abs(
                    closestSource.getPose().getRotation().minus(robot.getRotation()).getDegrees())
                < 80;
    return nearSource;
  }
}
