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
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class ReefAlign {

    public static List<Supplier<Pose2d>> generateWaypointsClosest(boolean useAdditionalOffset) {
        return generateWaypointsClosest(
                () -> DashboardUI.Overview.getControl()
                        .getReefLevelSwitchValue()
                        .toCoralLevel(),
                useAdditionalOffset);
    }

    public static List<Supplier<Pose2d>> generateWaypointsClosest(
            Supplier<CoralLevel> level, boolean useAdditionalOffset) {
        return generateWaypoints(
                () -> {
                    CoralPosition closestCoral = IGamePosition.closestTo(
                            RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values());

                    if (closestCoral
                                    .getPose(level.get())
                                    .getTranslation()
                                    .getDistance(RobotContainer.poseSensorFusion
                                            .getEstimatedPosition()
                                            .getTranslation())
                            > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return null;

                    return closestCoral;
                },
                level,
                useAdditionalOffset);
    }

    public static List<Supplier<Pose2d>> generateWaypoints(
            Supplier<CoralPosition> pole, Supplier<CoralLevel> level, boolean useAdditionalOffset) {
        return WaypointAlign.createWaypointsToTarget(
                () -> useAdditionalOffset
                        ? pole.get()
                                .getPose(level.get())
                                .transformBy(new Transform2d(-Constants.Align.ADDITIONAL_OFFSET, 0, Rotation2d.kZero))
                        : pole.get().getPose(level.get()),
                new ArrayList<Supplier<Transform2d>>() {
                    {
                        add(() -> level.get() == CoralLevel.L1
                                ? new Transform2d(0, -Constants.Align.L1_CLEARANCE_MIN, Rotation2d.kZero)
                                : new Transform2d(-Constants.Align.CLEARANCE_MIN, 0, Rotation2d.kZero));
                    }
                });
    }
}
