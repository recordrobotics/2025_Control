package frc.robot.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.utils.libraries.Elastic;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousLayout extends AbstractLayout {

  private static final Field2d field = new Field2d();
  private static LoggedDashboardChooser<Command> autoChooser;
  private static LoggedDashboardChooser<FieldStartingLocation> fieldStartingLocationChooser =
      new LoggedDashboardChooser<>("Starting Location");

  private static final Map<Integer, TuningData> velocityGraphData = new HashMap<>();

  public AutonomousLayout() {
    buildSendable("Field", field);

    EnumSet.allOf(FieldStartingLocation.class)
        .forEach(v -> fieldStartingLocationChooser.addOption(v.name(), v));
    fieldStartingLocationChooser.addDefaultOption(
        FieldStartingLocation.BargeCenter.name(), FieldStartingLocation.BargeCenter);

    addValueSendable("Velocity", () -> TuningData.MapToArray(velocityGraphData), "double[]");

    SmartDashboard.putBoolean("Autonomous/ResetLocationButton", false);
    SmartDashboard.putBoolean("Autonomous/LimelightRotation", false);
    SmartDashboard.putBoolean("Autonomous/ForceMT1", false);
  }

  public void setupAutoChooser() {
    autoChooser = new LoggedDashboardChooser<>("Auto Code", AutoBuilder.buildAutoChooser());
  }

  public void setRobotPose(Pose2d pose) {
    field.setRobotPose(pose);
  }

  public void setVisionPose(Pose2d pose) {
    field.getObject("Vision").setPose(pose);
  }

  public void putSwerveVelocityData(int id, double current, double target) {
    velocityGraphData.put(id, new TuningData(current, target));
  }

  @Override
  public void switchTo() {
    Elastic.selectTab("Autonomous");
  }

  @Override
  protected NetworkTable getNetworkTable() {
    return NetworkTableInstance.getDefault().getTable("/SmartDashboard/Autonomous");
  }

  public Command getAutoChooser() {
    return autoChooser.get();
  }

  public FieldStartingLocation getStartingLocation() {
    return fieldStartingLocationChooser.get() == null
        ? FieldStartingLocation.BargeCenter
        : fieldStartingLocationChooser.get();
  }

  public boolean getResetLocation() {
    return SmartDashboard.getBoolean("Autonomous/ResetLocationButton", false);
  }

  public boolean getLimelightRotation() {
    return SmartDashboard.getBoolean("Autonomous/LimelightRotation", false);
  }

  public boolean getForceMT1() {
    return SmartDashboard.getBoolean("Autonomous/ForceMT1", false);
  }
}
