package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private static KinematicState getKinematicStateForWaypoint(
            Pose2d previousWaypoint,
            Pose2d currentWaypoint,
            Pose2d nextWaypoint,
            double[] maxVelocity,
            double[] maxAcceleration,
            boolean useAcceleration) {
        double px = previousWaypoint.getX();
        double py = previousWaypoint.getY();
        double pr = previousWaypoint.getRotation().getRadians();

        double nx = nextWaypoint.getX();
        double ny = nextWaypoint.getY();
        double nr = nextWaypoint.getRotation().getRadians();

        double dpx = currentWaypoint.getX() - px;
        double dpy = currentWaypoint.getY() - py;
        double dpr = currentWaypoint.getRotation().getRadians() - pr;
        double dnx = nx - currentWaypoint.getX();
        double dny = ny - currentWaypoint.getY();
        double dnr = nr - currentWaypoint.getRotation().getRadians();

        double lp = Math.hypot(dpx, dpy);
        double ln = Math.hypot(dnx, dny);
        double t = lp < ln ? lp / (2 * ln) : (1 - ln / (2 * lp));

        double dx = dnx * t + dpx * (1 - t);
        double dy = dny * t + dpy * (1 - t);
        double dr = dnr * t + dpr * (1 - t);
        dr = MathUtil.clamp(dr, -maxVelocity[2], maxVelocity[2]);

        double dist = Math.hypot(dx, dy);
        if (dist > 1e-6) {
            double max_v = Math.hypot(maxVelocity[0], maxVelocity[1]);
            max_v /= Math.abs(dr);
            if (dist > max_v) {
                dx /= dist;
                dy /= dist;
                double v_peak = max_v;
                dx *= v_peak;
                dy *= v_peak;
            }
        } else {
            dx = 0.0;
            dy = 0.0;
        }

        double t_vx = 0, t_vy = 0, t_vr = 0;
        double t_ax = 0, t_ay = 0, t_ar = 0;

        if (useAcceleration) {
            dnr = MathUtil.clamp(dnr, -maxAcceleration[2], maxAcceleration[2]);
            double dist_a = Math.hypot(dnx, dny);
            if (dist_a > 1e-6) {
                double max_a = Math.hypot(maxAcceleration[0], maxAcceleration[1]);
                max_a /= Math.abs(dnr);
                if (dist_a > max_a) {
                    dnx /= dist_a;
                    dny /= dist_a;
                    double a_peak = max_a;
                    dnx *= a_peak;
                    dny *= a_peak;
                }
            } else {
                dnx = 0.0;
                dny = 0.0;
            }

            t_ax = dnx;
            t_ay = dny;
            t_ar = dnr;
        }

        t_vx = dx;
        t_vy = dy;
        t_vr = dr;

        return new KinematicState(
                new double[] {
                    currentWaypoint.getX(),
                    currentWaypoint.getY(),
                    currentWaypoint.getRotation().getRadians()
                }, // position
                new double[] {t_vx, t_vy, t_vr}, // velocity
                new double[] {t_ax, t_ay, t_ar} // acceleration
                );
    }

    public static Command align(Supplier<Pose2d>[] waypoints, Boolean[] includedWaypoints, Double[] waypointTimeouts) {
        return align(
                waypoints,
                includedWaypoints,
                waypointTimeouts,
                new double[] {4, 4, 4}, // max velocity
                new double[] {2, 2, 6.46}, // max acceleration
                new double[] {5, 5, 6.37} // max jerk
                );
    }

    public static Command align(
            Supplier<Pose2d>[] waypoints,
            Boolean[] includedWaypoints,
            Double[] waypointTimeouts,
            double[] maxVelocity,
            double[] maxAcceleration,
            double[] maxJerk) {

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
                            return getKinematicStateForWaypoint(
                                    index == 0
                                            ? RobotContainer.poseSensorFusion.getEstimatedPosition()
                                            : waypoints[index - 1].get(),
                                    waypoints[index].get(),
                                    waypoints[index + 1].get(),
                                    maxVelocity,
                                    maxAcceleration,
                                    true);
                        }
                    },
                    maxVelocity,
                    maxAcceleration,
                    maxJerk);
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
        return alignWithCommand(
                waypoints,
                waypointTimeouts,
                commandStartWaypoint,
                commandEndWaypoint,
                performCommand,
                new double[] {4, 4, 4}, // max velocity
                new double[] {2, 2, 6.46}, // max acceleration
                new double[] {5, 5, 6.37} // max jerk
                );
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
     * @param maxVelocity The maximum velocity for each axis
     * @param maxAcceleration The maximum acceleration for each axis
     * @param maxJerk The maximum jerk for each axis
     * @return
     */
    public static Command alignWithCommand(
            Supplier<Pose2d>[] waypoints,
            Double[] waypointTimeouts,
            int commandStartWaypoint,
            int commandEndWaypoint,
            Command performCommand,
            double[] maxVelocity,
            double[] maxAcceleration,
            double[] maxJerk) {

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

            double px = currentPose.getX();
            double py = currentPose.getY();

            double minDist = Double.MAX_VALUE;
            int closestWaypoint = -1;

            for (int i = 0; i < waypoints.length - 1; i++) {
                int waypointIndex = i;

                double wx = waypoints[i].get().getX();
                double wy = waypoints[i].get().getY();
                double wnx = waypoints[i + 1].get().getX();
                double wny = waypoints[i + 1].get().getY();

                double t = ((px - wx) * (wnx - wx) + (py - wy) * (wny - wy))
                        / ((wnx - wx) * (wnx - wx) + (wny - wy) * (wny - wy));
                if (i == 0 && t < 0) {
                    waypointIndex = -1;
                }
                t = MathUtil.clamp(t, 0, 1);
                double cx = wx + t * (wnx - wx);
                double cy = wy + t * (wny - wy);
                double dist = Math.hypot(cx - px, cy - py);
                if (dist < minDist) {
                    minDist = dist;
                    closestWaypoint = waypointIndex;
                }
            }

            return closestWaypoint;
        };

        return Commands.defer(
                        () -> {
                            for (int i = 0; i < waypoints.length; i++) {
                                // Only include waypoints before the command end and in front of the current
                                // waypoint (unless the current waypoint is after the command end - then back up
                                // first)
                                waypointsBeforeCommandEnd[i] = i <= commandEndWaypoint
                                        && i > Math.min(commandEndWaypoint - 1, currentWaypoint.get());
                            }

                            return align(
                                    waypoints,
                                    waypointsBeforeCommandEnd,
                                    waypointTimeouts,
                                    maxVelocity,
                                    maxAcceleration,
                                    maxJerk);
                        },
                        Set.of(RobotContainer.drivetrain))
                .alongWith(new WaitUntilCommand(() -> currentWaypoint.get() == commandStartWaypoint)
                        .andThen(performCommand))
                .andThen(align(
                        waypoints, waypointsAfterCommandEnd, waypointTimeouts, maxVelocity, maxAcceleration, maxJerk));
    }
}
