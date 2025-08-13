package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.AlgaePosition;
import java.util.function.Supplier;

public class AlgaeAlign {

    @SuppressWarnings("unchecked")
    public static Supplier<Pose2d>[] generateWaypoints(AlgaePosition pole) {
        return WaypointAlign.createWaypointsToTarget(
                () -> pole.getPose(), new Supplier[] {() -> new Transform2d(-0.3, 0, Rotation2d.kZero)});
    }
}
