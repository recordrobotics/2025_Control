package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;

public class PathAlign {
  public static Command createForReef(Pose2d target) {
    Pose2d
        currentPoseFromPoseSensorFusionButWithRotationBeingTheTargetDirectionOfTravelForThisPathPlannerPath =
            new Pose2d(
                RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation(),
                target
                    .minus(RobotContainer.poseSensorFusion.getEstimatedPosition())
                    .getRotation()); // Sometimes is wrong/goofy idk how to consistently reproduce
    // though

    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            currentPoseFromPoseSensorFusionButWithRotationBeingTheTargetDirectionOfTravelForThisPathPlannerPath,
            target);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            Constants.Align.PATH_CONSTRAINTS,
            null, // null for OTF paths.
            new GoalEndState(1.0, target.getRotation()) // holonomic rotation
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }
}
