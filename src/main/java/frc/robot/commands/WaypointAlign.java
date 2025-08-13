package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.Set;
import java.util.function.Supplier;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;

public class WaypointAlign {

    public static Supplier<Pose2d>[] createWaypointsToTarget(
            Supplier<Pose2d> target, Supplier<Transform2d>[] targetTransforms) {
        @SuppressWarnings("unchecked")
        Supplier<Pose2d>[] waypointSuppliers = new Supplier[targetTransforms.length + 1];

        waypointSuppliers[targetTransforms.length] = target;
        for (int i = 0; i < targetTransforms.length; i++) {
            final int index = i;
            waypointSuppliers[i] = () -> target.get().plus(targetTransforms[index].get());
        }

        return waypointSuppliers;
    }

    private static KinematicState getKinematicStateForWaypoint(Pose2d currentWaypoint, Pose2d nextWaypoint) {
        double dx = nextWaypoint.getX() - currentWaypoint.getX();
        double dy = nextWaypoint.getY() - currentWaypoint.getY();
        double dtheta = nextWaypoint.getRotation().getRadians()
                - currentWaypoint.getRotation().getRadians();

        return new KinematicState(
                new double[] {
                    currentWaypoint.getX(),
                    currentWaypoint.getY(),
                    currentWaypoint.getRotation().getRadians()
                }, // position
                new double[] {0, 0, 0}, // velocity
                new double[] {0, 0, 0} // acceleration
                );
    }

    public static Command align(Supplier<Pose2d>[] waypoints, Boolean[] includedWaypoints, Double[] waypointTimeouts) {

        if (waypoints.length > waypointTimeouts.length) {
            throw new IllegalArgumentException("Waypoints and waypoint timeouts must have the same length");
        }

        if (waypoints.length > includedWaypoints.length) {
            throw new IllegalArgumentException("Waypoints and included waypoints must have the same length");
        }

        SequentialCommandGroup waypointGroup = new SequentialCommandGroup();
        for (int i = 0; i < waypoints.length; i++) {
            if (!includedWaypoints[i]) continue;

            final int index = i;
            RuckigAlign cmd = new RuckigAlign(
                    () -> {
                        if (index == waypoints.length - 1 || !includedWaypoints[index + 1]) {
                            return new KinematicState(
                                    new double[] {
                                        waypoints[index].get().getX(),
                                        waypoints[index].get().getY(),
                                        waypoints[index].get().getRotation().getRadians()
                                    },
                                    new double[] {0, 0, 0},
                                    new double[] {0, 0, 0});
                        } else {
                            return getKinematicStateForWaypoint(waypoints[index].get(), waypoints[index + 1].get());
                        }
                    },
                    new double[] {4, 4, 4},
                    new double[] {2, 2, 6.46},
                    new double[] {5, 5, 6.37});
            waypointGroup.addCommands(cmd.withTimeout(waypointTimeouts[i]));
        }

        return waypointGroup;
    }

    /**
     * Aligns the robot with a command that runs at a specific waypoint. The command will run
     * immediately when the robot reaches the waypoint specified by {@code commandStartWaypoint} and
     * will wait until the robot reaches the waypoint specified by {@code commandEndWaypoint} before
     * continuing to the next waypoint.
     *
     * @param waypoints The waypoints to align with
     * @param waypointTimeouts The timeouts for each waypoint
     * @param commandStartWaypoint The waypoint when to run the command
     * @param commandEndWaypoint The waypoint before which to wait for command to finish (if start is
     *     -1 and end is 0 then the command will run immediately and wait before entering the segment
     *     between first and second waypoint)
     * @param performCommand The command to run when at the right waypoint
     * @return
     */
    public static Command alignWithCommand(
            Supplier<Pose2d>[] waypoints,
            Double[] waypointTimeouts,
            int commandStartWaypoint,
            int commandEndWaypoint,
            Command performCommand) {

        if (waypoints.length > waypointTimeouts.length) {
            throw new IllegalArgumentException("Waypoints and waypoint timeouts must have the same length");
        }

        Boolean[] waypointsBeforeCommandEnd = new Boolean[waypoints.length];

        Boolean[] waypointsAfterCommandEnd = new Boolean[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            waypointsAfterCommandEnd[i] = i > commandEndWaypoint;
        }

        Supplier<Integer> currentWaypoint = () -> {
            Pose2d currentPose = RobotContainer.poseSensorFusion.getEstimatedPosition();

            // Find closest waypoint
            int closestWaypoint = 0;
            double minDist = Double.MAX_VALUE;
            for (int i = 0; i < waypoints.length; i++) {
                double dx = currentPose.getX() - waypoints[i].get().getX();
                double dy = currentPose.getY() - waypoints[i].get().getY();
                double dist = dx * dx + dy * dy;
                if (dist < minDist) {
                    minDist = dist;
                    closestWaypoint = i;
                }
            }

            // Check if near any segment between waypoints
            double segmentThreshold = 0.5 * 0.5; // 0.5 meters squared, adjust as needed
            for (int i = 0; i < waypoints.length - 1; i++) {
                Pose2d p1 = waypoints[i].get();
                Pose2d p2 = waypoints[i + 1].get();

                // Project currentPose onto segment p1-p2
                double x1 = p1.getX(), y1 = p1.getY();
                double x2 = p2.getX(), y2 = p2.getY();
                double x0 = currentPose.getX(), y0 = currentPose.getY();

                double dx = x2 - x1;
                double dy = y2 - y1;
                double lengthSq = dx * dx + dy * dy;
                if (lengthSq == 0) continue; // skip degenerate segment

                double t = ((x0 - x1) * dx + (y0 - y1) * dy) / lengthSq;
                if (t >= 0 && t <= 1) {
                    // Closest point on segment
                    double projX = x1 + t * dx;
                    double projY = y1 + t * dy;
                    double distSq = (x0 - projX) * (x0 - projX) + (y0 - projY) * (y0 - projY);
                    if (distSq < segmentThreshold) {
                        return i; // Near segment i (between waypoints i and i+1)
                    }
                }
            }

            // If closest to first waypoint and not near first segment
            if (closestWaypoint == 0) {
                return -1;
            }
            // If closest to last waypoint and not near last segment
            if (closestWaypoint == waypoints.length - 1) {
                return waypoints.length - 1;
            }
            // Otherwise, return closest waypoint index
            return closestWaypoint;
        };

        return new InstantCommand(() -> {
                    for (int i = 0; i < waypoints.length; i++) {
                        // Only include waypoints before the command end and in front of the current
                        // waypoint (unless the current waypoint is after the command end - then back up
                        // first)
                        waypointsBeforeCommandEnd[i] =
                                i <= commandEndWaypoint && i > Math.min(commandEndWaypoint - 1, currentWaypoint.get());
                    }
                })
                .andThen(Commands.defer(
                        () -> align(waypoints, waypointsBeforeCommandEnd, waypointTimeouts),
                        Set.of(RobotContainer.drivetrain)))
                .alongWith(new WaitUntilCommand(() -> currentWaypoint.get() == commandStartWaypoint)
                        .andThen(performCommand))
                .andThen(align(waypoints, waypointsAfterCommandEnd, waypointTimeouts));
    }
}
