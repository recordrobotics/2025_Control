package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.control.AbstractControl;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class OverviewLayout extends AbstractLayout {

  public enum DriverOrientation {
    XAxisTowardsTrigger("Competition"),
    YAxisTowardsTrigger("Y Axis"),
    XAxisInvertTowardsTrigger("Couch Drive");

    public final String display_name;

    DriverOrientation(String orientation) {
      display_name = orientation;
    }
  }

  private static SendableChooser<DriverOrientation> driverOrientation =
      new SendableChooser<DriverOrientation>();
  private static SendableChooser<AbstractControl> driveMode =
      new SendableChooser<AbstractControl>();
  private static AbstractControl _defaultControl;

  // private Supplier<Boolean> poseCertainValue = () -> false;
  private Supplier<Boolean> compressorValue = () -> false;
  private Supplier<Boolean> compressorManuallyDisabledValue = () -> false;
  private Supplier<Boolean> navSensorValue = () -> false;
  private Supplier<Integer> tagNumValue = () -> 0;
  private Supplier<Double> confidenceValue = () -> 0.0;
  private Supplier<Boolean> hasVisionValue = () -> false;
  private Supplier<Boolean> limelightConnectedValue = () -> false;

  private static final Map<Integer, TuningData> shooterSpeedData = new HashMap<>();

  public OverviewLayout() {
    // getTab()
    //         .addBoolean("Pose Certain", () -> poseCertainValue.get())
    //         .withWidget(BuiltInWidgets.kBooleanBox)
    //         .withSize(1, 1)
    //         .withPosition(0, 1);

    getTab()
        .addBoolean("Compressor", () -> compressorValue.get())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(7, 0)
        .withSize(1, 1);

    getTab()
        .addBoolean("CP Disabled", () -> compressorManuallyDisabledValue.get())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(8, 0)
        .withSize(1, 1);

    getTab()
        .addBoolean("Nav Sensor", () -> navSensorValue.get())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(6, 1)
        .withSize(1, 1);

    getTab()
        .addInteger("Tag Count", () -> tagNumValue.get())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(7, 1)
        .withSize(1, 1);

    getTab()
        .addDouble("Confidence", () -> confidenceValue.get())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(8, 1)
        .withSize(1, 1);

    getTab()
        .addBoolean("Has Vision", () -> hasVisionValue.get())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(7, 2)
        .withSize(1, 1);

    getTab()
        .addBoolean("Limelight Connected", () -> limelightConnectedValue.get())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(8, 2)
        .withSize(1, 1);
  }

  /**
   * Initializes the control object
   *
   * @param defaultControl the first term will always be the default control object
   * @param controls any other control objects you want to initialize
   */
  public void addControls(AbstractControl defaultControl, AbstractControl... controls) {
    _defaultControl = defaultControl;

    // Sets up drive mode options
    for (AbstractControl abstractControl : controls) {
      driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
    }
    driveMode.setDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

    EnumSet.allOf(DriverOrientation.class)
        .forEach(v -> driverOrientation.addOption(v.display_name, v));
    driverOrientation.setDefaultOption(
        DriverOrientation.XAxisTowardsTrigger.display_name, DriverOrientation.XAxisTowardsTrigger);

    // Creates the UI for driverOrientation
    getTab()
        .add("Driver Orientation", driverOrientation)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 0)
        .withSize(3, 1);

    // Creates the UI for drive mode
    getTab()
        .add("Drive Mode", driveMode)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 1)
        .withSize(3, 1);
  }

  public void setCompressor(Supplier<Boolean> compressor) {
    compressorValue = compressor;
  }

  public void setCompressorManuallyDisabled(Supplier<Boolean> compressorManuallyDisabled) {
    compressorManuallyDisabledValue = compressorManuallyDisabled;
  }

  public void setNavSensor(Supplier<Boolean> navSensor) {
    navSensorValue = navSensor;
  }

  public void setTagNum(Supplier<Integer> tagNum) {
    tagNumValue = tagNum;
  }

  public void setConfidence(Supplier<Double> confidence) {
    confidenceValue = confidence;
  }

  public void setHasVision(Supplier<Boolean> hasVision) {
    hasVisionValue = hasVision;
  }

  public void setLimelightConnected(Supplier<Boolean> limelightConnected) {
    limelightConnectedValue = limelightConnected;
  }

  public void putShooterSpeedData(int id, double current, double target) {
    shooterSpeedData.put(id, new TuningData(current, target));
  }

  @Override
  public ShuffleboardTab getTab() {
    return Shuffleboard.getTab("Overview");
  }

  public DriverOrientation getDriverOrientation() {
    return driverOrientation.getSelected();
  }

  public AbstractControl getControl() {
    if (driveMode.getSelected() == null) return _defaultControl;
    return driveMode.getSelected();
  }
}
