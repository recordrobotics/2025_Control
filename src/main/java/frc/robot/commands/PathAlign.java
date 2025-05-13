package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.List;

public class PathAlign {
  public Command createForReef(Pose2d target) {
    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            RobotContainer.poseSensorFusion.getEstimatedPosition(),
            target.transformBy(new Transform2d(-0.2, 0, Rotation2d.kZero)),
            target);

    PathConstraints constraints = new PathConstraints(2.0, 2.0, 1 * Math.PI, 2 * Math.PI);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // null for OTF paths.
            new GoalEndState(0.0, target.getRotation()) // holonomic rotation
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }
}
