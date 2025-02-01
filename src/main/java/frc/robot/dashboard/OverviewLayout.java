package frc.robot.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.control.AbstractControl;
import frc.robot.utils.libraries.Elastic;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  private static LoggedDashboardChooser<DriverOrientation> driverOrientation =
      new LoggedDashboardChooser<DriverOrientation>("Driver Orientation");
  private static LoggedDashboardChooser<AbstractControl> driveMode =
      new LoggedDashboardChooser<AbstractControl>("Drive Mode");
  private static AbstractControl _defaultControl;

  // private Supplier<Boolean> poseCertainValue = () -> false;
  private Supplier<Boolean> navSensorValue = () -> false;
  private Supplier<Integer> tagNumValue = () -> 0;
  private Supplier<Double> confidenceValue = () -> 0.0;
  private Supplier<Boolean> hasVisionValue = () -> false;
  private Supplier<Boolean> limelightConnectedValue = () -> false;

  private static final Map<Integer, TuningData> shooterSpeedData = new HashMap<>();

  public OverviewLayout() {
    addValueSendable("Nav Sensor", () -> navSensorValue.get(), "boolean");

    addValueSendable("Tag Count", () -> tagNumValue.get(), "int");

    addValueSendable("Confidence", () -> confidenceValue.get(), "double");

    addValueSendable("Has Vision", () -> hasVisionValue.get(), "boolean");

    addValueSendable("Limelight Connected", () -> limelightConnectedValue.get(), "boolean");
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
    driveMode.addDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

    EnumSet.allOf(DriverOrientation.class)
        .forEach(v -> driverOrientation.addOption(v.display_name, v));
    driverOrientation.addDefaultOption(
        DriverOrientation.XAxisTowardsTrigger.display_name, DriverOrientation.XAxisTowardsTrigger);
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
  public void switchTo() {
    Elastic.selectTab("Overview");
  }

  @Override
  protected NetworkTable getNetworkTable() {
    return NetworkTableInstance.getDefault().getTable("/SmartDashboard/Overview");
  }

  public DriverOrientation getDriverOrientation() {
    return driverOrientation.get();
  }

  public AbstractControl getControl() {
    if (driveMode.get() == null) return _defaultControl;
    return driveMode.get();
  }
}
