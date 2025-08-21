package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.AlgaePosition;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class AlgaeAlign {

    public static List<Supplier<Pose2d>> generateWaypoints(AlgaePosition pole) {
        return WaypointAlign.createWaypointsToTarget(() -> pole.getPose(), new ArrayList<Supplier<Transform2d>>() {
            {
                add(() -> new Transform2d(-0.3, 0, Rotation2d.kZero));
            }
        });
    }
}
