package frc.robot.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.commands.auto.BargeLeftAuto;
import frc.robot.commands.auto.BargeRightAuto;
import frc.robot.utils.libraries.Elastic;
import java.io.IOException;
import java.util.EnumSet;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@SuppressWarnings("java:S2325")
public final class AutonomousLayout extends AbstractLayout {

    private final Field2d field = new Field2d();
    private LoggedDashboardChooser<Command> autoChooser;
    private LoggedDashboardChooser<FieldStartingLocation> fieldStartingLocationChooser =
            new LoggedDashboardChooser<>("Starting Location");

    public AutonomousLayout() {
        buildSendable("Field", field);

        EnumSet.allOf(FieldStartingLocation.class)
                .forEach(v -> fieldStartingLocationChooser.addOption(v.toString(), v));
        fieldStartingLocationChooser.addDefaultOption(
                FieldStartingLocation.BARGE_CENTER.toString(), FieldStartingLocation.BARGE_CENTER);

        SmartDashboard.putBoolean("Autonomous/ResetLocationButton", false);
        SmartDashboard.putBoolean("Autonomous/LimelightRotation", false);
        SmartDashboard.putBoolean("Autonomous/EncoderReset", false);
        SmartDashboard.putBoolean("Autonomous/ForceMT1", false);
    }

    public void setupAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("Auto Code", AutoBuilder.buildAutoChooser());
        try {
            autoChooser.addOption("CMD_BargeLeftOuter", new BargeLeftAuto());
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        try {
            autoChooser.addOption("CMD_BargeRightOuter", new BargeRightAuto());
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    public void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setVisionPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
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
                ? FieldStartingLocation.BARGE_CENTER
                : fieldStartingLocationChooser.get();
    }

    public boolean isResetLocationPressed() {
        return SmartDashboard.getBoolean("Autonomous/ResetLocationButton", false);
    }

    public boolean isLimelightRotationPressed() {
        return SmartDashboard.getBoolean("Autonomous/LimelightRotation", false);
    }

    public boolean isEncoderResetPressed() {
        return SmartDashboard.getBoolean("Autonomous/EncoderReset", false);
    }

    public boolean isForceMT1Pressed() {
        return SmartDashboard.getBoolean("Autonomous/ForceMT1", false);
    }
}
