package frc.robot.utils;

import java.util.EnumSet;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ShuffleboardUI;

public class DriverStationUtils {
    public enum FieldStartingLocation {
        FrontSpeaker(
                // Red
                new Pose2d(14.169, 5.584, Rotation2d.fromDegrees(180)),
                // Blue
                new Pose2d(2.371, 5.584, Rotation2d.fromDegrees(0))),
        AtAmp(
                // Red
                new Pose2d(14.89, 7.27, Rotation2d.fromDegrees(-90)),
                // Blue
                new Pose2d(1.65, 7.27, Rotation2d.fromDegrees(-90))),
        DiagonalSpeaker(
                // Red
                new Pose2d(16.268, 4.454, Rotation2d.fromDegrees(-120.665)),
                // Blue
                new Pose2d(1.564, 4.403, Rotation2d.fromDegrees(-60.849))),
                ;

        private final Pose2d m_transformRed;
        private final Pose2d m_transformBlue;

        private FieldStartingLocation(Pose2d poseRed, Pose2d poseBlue) {
            m_transformRed = poseRed;
            m_transformBlue = poseBlue;
        }

        public Pose2d getPose() {
            return getCurrentAlliance() == Alliance.Red ? m_transformRed : m_transformBlue;
        }
    }

    public static SendableChooser<FieldStartingLocation> fieldStartingLocationChooser = new SendableChooser<>();

    static {

        EnumSet.allOf(FieldStartingLocation.class)
                .forEach(v -> fieldStartingLocationChooser.addOption(v.toString(), v));
        fieldStartingLocationChooser.setDefaultOption(FieldStartingLocation.AtAmp.toString(),
                FieldStartingLocation.AtAmp);

        ShuffleboardTab tab = ShuffleboardUI.Autonomous.getTab();
        var fieldStartingLocationWidget = tab.add("Starting Location", fieldStartingLocationChooser);
        fieldStartingLocationWidget.withWidget(BuiltInWidgets.kSplitButtonChooser);
        fieldStartingLocationWidget.withSize(3, 1);
        fieldStartingLocationWidget.withPosition(6, 0);
    }

    public static Alliance getCurrentAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get();
        }
        return Alliance.Blue;
    }

    public static FieldStartingLocation getStartingLocation() {
        return fieldStartingLocationChooser.getSelected();
    }
}