package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import java.util.List;

public class ReefAlign {

    public static List<Pose2d> generateWaypointsClosest(boolean useAdditionalOffset) {
        return generateWaypointsClosest(
                DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel(), useAdditionalOffset);
    }

    public static List<Pose2d> generateWaypointsClosest(CoralLevel level, boolean useAdditionalOffset) {
        CoralPosition closestCoral =
                IGamePosition.closestTo(RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values());

        if (closestCoral
                        .getPose(level)
                        .getTranslation()
                        .getDistance(RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation())
                > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return List.of();

        return generateWaypoints(closestCoral, level, useAdditionalOffset);
    }

    public static List<Pose2d> generateWaypoints(CoralPosition pole, CoralLevel level, boolean useAdditionalOffset) {
        return WaypointAlign.createWaypointsToTarget(
                useAdditionalOffset
                        ? pole.getPose(level)
                                .transformBy(new Transform2d(-Constants.Align.ADDITIONAL_OFFSET, 0, Rotation2d.kZero))
                        : pole.getPose(level),
                new Transform2d[] {
                    level == CoralLevel.L1
                            ? new Transform2d(0, -Constants.Align.L1_CLEARANCE_MIN, Rotation2d.kZero)
                            : new Transform2d(-Constants.Align.CLEARANCE_MIN, 0, Rotation2d.kZero)
                });
    }
}
