package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class DoubleXboxSpin extends AbstractControl {

  private XboxController drivebox;
  private XboxController notesbox;
  private double speed_level = 0.1;

  public DoubleXboxSpin(int driveboxID, int notesboxID) {
    // Sets up xbox controllers
    drivebox = new XboxController(driveboxID);
    notesbox = new XboxController(notesboxID);
    // Sets triggers that map to speeds
    setSpeedTriggers();
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

  public Double getSpin() {
    return SimpleMath.ApplyThresholdAndSensitivity(
        -drivebox.getRawAxis(4),
        Constants.Control.XBOX_SPIN_ROT_THRESHOLD,
        Constants.Control.XBOX_SPIN_ROT_SENSITIVITY);
  }

  public Double getDirectionalSpeedLevel() {
    return speed_level;
  }

  public Double getSpinSpeedLevel() {
    return 0.5 * speed_level;
  }

  @Override
  public Boolean getPoseReset() {
    return drivebox.getRawButtonPressed(7);
  }

  @Override
  public Boolean getKill() {
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
}
