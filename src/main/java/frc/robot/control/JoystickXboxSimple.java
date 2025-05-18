package frc.robot.control;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.assists.DrivetrainControl;

public class JoystickXboxSimple extends AbstractControl {

  private Joystick joystick;
  private XboxController xbox_controller;

  private ReefLevelSwitchValue reefswitch = ReefLevelSwitchValue.L4;

  public JoystickXboxSimple(int joystickPort, int xboxPort) {
    joystick = new Joystick(joystickPort);
    xbox_controller = new XboxController(xboxPort);

    new Trigger(() -> xbox_controller.getAButtonPressed())
        .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L1));
    new Trigger(() -> xbox_controller.getXButtonPressed())
        .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L2));
    new Trigger(() -> xbox_controller.getBButtonPressed())
        .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L3));
    new Trigger(() -> xbox_controller.getYButtonPressed())
        .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L4));
  }

  private Transform2d lastVelocity = new Transform2d();
  private Transform2d lastAcceleration = new Transform2d();

  @Override
  public DrivetrainControl getDrivetrainControl() {
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

    Transform2d velocity = new Transform2d(x, y, new Rotation2d(getSpin() * getSpinSpeedLevel()));
    Transform2d acceleration =
        new Transform2d(
                velocity.getTranslation().minus(lastVelocity.getTranslation()).div(0.02),
                velocity.getRotation().minus(lastVelocity.getRotation()))
            .div(0.02);
    Transform2d jerk =
        new Transform2d(
                acceleration.getTranslation().minus(lastAcceleration.getTranslation()).div(0.02),
                acceleration.getRotation().minus(lastAcceleration.getRotation()))
            .div(0.02);

    lastVelocity = velocity;
    lastAcceleration = acceleration;

    if (getElevatorRelativeDrive() || getCoralIntakeRelativeDrive()) {
      return DrivetrainControl.createRobotRelative(velocity, acceleration, jerk);
    } else {
      return DrivetrainControl.createFieldRelative(
          velocity,
          acceleration,
          jerk,
          RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());
    }
  }

  public Boolean getAutoAlign() {
    return false;
  }

  public Boolean getAutoAlignNear() {
    return false;
  }

  public Boolean getElevatorRelativeDrive() {
    return joystick.getRawButton(8)
        || (getAutoScore()
            && getReefLevelSwitchValue()
                != ReefLevelSwitchValue.L1); // elevator relative when auto score
  }

  public Boolean getCoralIntakeRelativeDrive() {
    return joystick.getRawButton(10)
        || (getAutoScore()
            && getReefLevelSwitchValue()
                == ReefLevelSwitchValue.L1); // coral relative when auto score
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
    return getAutoScore(); // half speed auto enabled when scoring
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
    return joystick.getRawButtonPressed(5);
  }

  @Override
  public Boolean getLimelightReset() {
    return joystick.getRawButtonPressed(6);
  }

  @Override
  public Boolean getKill() {
    return joystick.getRawButton(4);
  }

  @Override
  public void vibrate(RumbleType type, double value) {
    xbox_controller.setRumble(type, value);
  }

  @Override
  public Boolean getAutoScore() {
    return joystick.getRawButton(1);
  }

  @Override
  public Boolean getElevatorL2() {
    return false;
  }

  @Override
  public Boolean getElevatorL3() {
    return false;
  }

  @Override
  public Boolean getElevatorL4() {
    return false;
  }

  @Override
  public Boolean getCoralShoot() {
    return false;
  }

  @Override
  public Boolean getCoralGroundIntake() {
    return false;
  }

  @Override
  public Boolean getCoralGroundIntakeSimple() {
    return joystick.getRawButton(2);
  }

  @Override
  public Boolean getReefAlgaeSimple() {
    return joystick.getRawButton(3);
  }

  @Override
  public Boolean getCoralSourceIntake() {
    return false;
  }

  @Override
  public Boolean getElevatorAlgaeLow() {
    return false;
  }

  @Override
  public Boolean getElevatorAlgaeHigh() {
    return false;
  }

  @Override
  public ReefLevelSwitchValue getReefLevelSwitchValue() {
    return reefswitch;
  }

  @Override
  public Boolean getManualOverride() {
    return false;
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
    return false;
  }

  @Override
  public LinearVelocity getManualElevatorVelocity() {
    double axis = SimpleMath.povToVector(joystick.getPOV()).getY();
    return Centimeters.of(50).per(Seconds).times(axis);
  }

  @Override
  public AngularVelocity getManualElevatorArmVelocity() {
    double axis = SimpleMath.povToVector(joystick.getPOV()).getY();
    return Degrees.of(180).per(Seconds).times(axis);
  }

  @Override
  public Boolean getClimb() {
    return joystick.getRawButton(7);
  }

  @Override
  public Boolean getCoralSourceIntakeAuto() {
    Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
    SourcePosition closestSource = IGamePosition.closestTo(robot, SourcePosition.values());

    boolean nearSource =
        closestSource.getPose().getTranslation().getDistance(robot.getTranslation()) < 2.3
            && Math.abs(
                    closestSource.getPose().getRotation().minus(robot.getRotation()).getDegrees())
                < 80;
    return nearSource;
  }
}
