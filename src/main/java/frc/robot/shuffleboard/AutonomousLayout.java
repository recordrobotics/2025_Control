package frc.robot.shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoName;
import frc.robot.Constants.FieldStartingLocation;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class AutonomousLayout extends AbstractLayout {

  private static final Field2d field = new Field2d();
  private static SendableChooser<AutoName> autoChooser = new SendableChooser<>();
  private static SendableChooser<FieldStartingLocation> fieldStartingLocationChooser =
      new SendableChooser<>();

  private static final Map<Integer, TuningData> velocityGraphData = new HashMap<>();

  public AutonomousLayout() {
    getTab().add(field).withWidget(BuiltInWidgets.kField).withSize(6, 3).withPosition(0, 0);

    EnumSet.allOf(AutoName.class).forEach(v -> autoChooser.addOption(v.pathref, v));
    autoChooser.setDefaultOption(AutoName._4NoteSpeaker.pathref, AutoName._4NoteSpeaker);

    // Creates the UI for auto routines
    getTab()
        .add("Auto Code", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(3, 1)
        .withPosition(6, 1);

    EnumSet.allOf(FieldStartingLocation.class)
        .forEach(v -> fieldStartingLocationChooser.addOption(v.name(), v));
    fieldStartingLocationChooser.setDefaultOption(
        FieldStartingLocation.FrontSpeakerClose.name(), FieldStartingLocation.FrontSpeakerClose);

    // Creates the UI for starting location
    getTab()
        .add("Starting Location", fieldStartingLocationChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withSize(6, 1)
        .withPosition(0, 3);
    getTab()
        .addDoubleArray("Velocity", () -> TuningData.MapToArray(velocityGraphData))
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(6, 2)
        .withSize(4, 3);
  }

  public void setRobotPose(Pose2d pose) {
    field.setRobotPose(pose);
  }

  public void setVisionPose(Pose2d pose) {
    field.getObject("Vision").setPose(pose);
    ;
  }

  public void putSwerveVelocityData(int id, double current, double target) {
    velocityGraphData.put(id, new TuningData(current, target));
  }

  @Override
  public ShuffleboardTab getTab() {
    return Shuffleboard.getTab("Autonomous");
  }

  public String getAutoChooser() {
    return autoChooser.getSelected().pathref;
  }

  public FieldStartingLocation getStartingLocation() {
    if (fieldStartingLocationChooser.getSelected() == null)
      return FieldStartingLocation.FrontSpeaker;
    return fieldStartingLocationChooser.getSelected();
  }
}
