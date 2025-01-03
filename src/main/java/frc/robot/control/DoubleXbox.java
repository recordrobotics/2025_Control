package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.PoseTracker;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class DoubleXbox extends AbstractControl {

  private XboxController drivebox;
  private XboxController notesbox;
  private double speed_level = 0.1;
  private PIDController anglePID = new PIDController(3.36, 0, 0);

  public DoubleXbox(int driveboxID, int notesboxID) {
    drivebox = new XboxController(driveboxID);
    notesbox = new XboxController(notesboxID);
    // Sets triggers that map to speeds
    setSpeedTriggers();
    // Sets up PID
    anglePID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public DriveCommandData getDriveCommandData() {

    // Calculates spin
    double robot_angle = PoseTracker.getEstimatedPosition().getRotation().getRadians();
    double target_angle = super.OrientAngle(getAngle().getFirst()).getRadians();
    double spin = anglePID.calculate(robot_angle, target_angle);
    // Calculates proportion of PID to multiply by
    double magnitude = getAngle().getSecond();

    // Gets information needed to drive
    DriveCommandData driveCommandData =
        new DriveCommandData(
            getXY().getFirst() * getDirectionalSpeedLevel(),
            getXY().getSecond() * getDirectionalSpeedLevel(),
            spin * magnitude,
            true);

    // Returns
    return driveCommandData;
  }

  public void setSpeedTriggers() {
    new Trigger(drivebox::getAButton)
        .onTrue(new InstantCommand(() -> speed_level = 0.1 * Constants.Swerve.robotMaxSpeed));
    new Trigger(drivebox::getBButton)
        .onTrue(new InstantCommand(() -> speed_level = 0.2 * Constants.Swerve.robotMaxSpeed));
    new Trigger(drivebox::getXButton)
        .onTrue(new InstantCommand(() -> speed_level = 0.35 * Constants.Swerve.robotMaxSpeed));
    new Trigger(drivebox::getYButton)
        .onTrue(new InstantCommand(() -> speed_level = 0.6 * Constants.Swerve.robotMaxSpeed));
  }

  public Pair<Double, Double> getXY() {
    double X =
        SimpleMath.ApplyThresholdAndSensitivity(
            drivebox.getRawAxis(0),
            Constants.Control.XBOX_X_THRESHOLD,
            Constants.Control.XBOX_DIRECTIONAL_SENSITIVITY);
    double Y =
        SimpleMath.ApplyThresholdAndSensitivity(
            drivebox.getRawAxis(1),
            Constants.Control.XBOX_X_THRESHOLD,
            Constants.Control.XBOX_DIRECTIONAL_SENSITIVITY);
    return super.OrientXY(new Pair<Double, Double>(X, Y));
  }

  public Pair<Rotation2d, Double> getAngle() {

    // Gets x and y axis of xbox
    double x_axis = drivebox.getRawAxis(4);
    double y_axis = drivebox.getRawAxis(5);

    // Gets magnitude of xbox axis
    double magnitude = Math.sqrt(x_axis * x_axis + y_axis * y_axis);

    // Calculates magnitude of control and adjusted angle
    double proportion =
        SimpleMath.ApplyThresholdAndSensitivity(
            magnitude, Constants.Control.XBOX_SPIN_THRESHOLD, 1);
    Rotation2d adjusted_angle = super.OrientAngle(new Rotation2d(-Math.atan2(y_axis, x_axis)));

    // Returns pair
    return new Pair<Rotation2d, Double>(adjusted_angle, proportion);
  }

  public Double getDirectionalSpeedLevel() {
    return speed_level;
  }

  @Override
  public Boolean getPoseReset() {
    return drivebox.getRawButtonPressed(7);
  }

  @Override
  public Boolean getKillAuto() {
    return drivebox.getRawButton(8) || notesbox.getRawButton(8);
  }

  @Override
  public Boolean getKillCompressor() {
    return drivebox.getPOV() == 180 || notesbox.getPOV() == 180;
  }

  @Override
  public void vibrate(double value) {
    drivebox.setRumble(RumbleType.kBothRumble, value);
    notesbox.setRumble(RumbleType.kBothRumble, value);
  }

  @Override
  public Boolean getAcquire() {
    return notesbox.getLeftTriggerAxis() > 0.3;
  }

  @Override
  public Boolean getReverse() {
    return notesbox.getLeftBumper();
  }

  @Override
  public Boolean getShootSpeaker() {
    return notesbox.getRightTriggerAxis() > 0.3;
  }

  @Override
  public Boolean getShootAmp() {
    return notesbox.getRightBumper();
  }

  @Override
  public Boolean getManualShootSpeaker() {
    return notesbox.getAButton();
  }

  @Override
  public Boolean getManualShootAmp() {
    return notesbox.getBButton();
  }

  @Override
  public Boolean getManualChannel() {
    return notesbox.getXButton();
  }

  @Override
  public Boolean getManualCrashbar() {
    return notesbox.getYButton();
  }

  @Override
  public Boolean getManualClimbers() {
    return notesbox.getRawButton(7);
  }

  @Override
  public Boolean getTeleAmp() {
    return false;
  }

  @Override
  public Boolean getTeleSpeaker() {
    return false;
  }

  @Override
  public Boolean getTeleChain() {
    return false;
  }
}
