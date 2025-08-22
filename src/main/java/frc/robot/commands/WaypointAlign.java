package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.RuckigAlign.AlignMode;
import frc.robot.commands.RuckigAlign.RuckigAlignGroup;
import frc.robot.commands.RuckigAlign.RuckigAlignState;
import frc.robot.utils.SimpleMath;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;

public class WaypointAlign {

    public static List<Pose2d> createWaypointsToTarget(Pose2d target, Transform2d[] targetTransforms) {

        List<Pose2d> waypoints = new ArrayList<Pose2d>(targetTransforms.length + 1);

        for (Transform2d transform : targetTransforms) {
            waypoints.add(target.plus(transform));
        }
        waypoints.add(target);

        return waypoints;
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
            if (Math.abs(dr) > 1e-6) {
                max_v /= Math.abs(dr);
            }
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
                if (Math.abs(dnr) > 1e-6) {
                    max_a /= Math.abs(dnr);
                }
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

    private static int getCurrentWaypoint(List<Pose2d> waypoints) {
        Pose2d currentPose = RobotContainer.poseSensorFusion.getEstimatedPosition();

        double px = currentPose.getX();
        double py = currentPose.getY();

        double minDist = Double.MAX_VALUE;
        int closestWaypoint = -1;

        for (int i = 0; i < waypoints.size() - 1; i++) {
            int waypointIndex = i;

            Pose2d wp = waypoints.get(i);
            Pose2d wpn = waypoints.get(i + 1);
            double wx = wp.getX();
            double wy = wp.getY();
            double wnx = wpn.getX();
            double wny = wpn.getY();

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
    }

    private static double moduleToRobotRadians(double moduleValue) {
        return moduleValue / Constants.Swerve.locDist;
    }

    public record KinematicConstraints(double[] maxVelocity, double[] maxAcceleration, double[] maxJerk) {
        public static final KinematicConstraints DEFAULT = new KinematicConstraints(
                new double[] {
                    Constants.Align.MAX_VELOCITY / SimpleMath.SQRT2,
                    Constants.Align.MAX_VELOCITY / SimpleMath.SQRT2,
                    Constants.Align.MAX_ANGULAR_VELOCITY
                }, // max velocity
                new double[] {
                    Constants.Align.MAX_ACCELERATION / SimpleMath.SQRT2,
                    Constants.Align.MAX_ACCELERATION / SimpleMath.SQRT2,
                    moduleToRobotRadians(Constants.Align.MAX_ACCELERATION / SimpleMath.SQRT2)
                }, // max acceleration
                new double[] {
                    Constants.Align.MAX_JERK / SimpleMath.SQRT2,
                    Constants.Align.MAX_JERK / SimpleMath.SQRT2,
                    moduleToRobotRadians(Constants.Align.MAX_JERK / SimpleMath.SQRT2)
                } // max jerk
                );
    }

    /**
     * Aligns the robot to a target pose. The robot will move to the target pose and stop there.
     * @param target The target pose to align to
     * @param timeout The timeout for the alignment
     * @return A command that aligns the robot to the target
     */
    public static Command align(Pose2d target, double timeout) {
        return align(List.of(target), 0, 0, true, new Double[] {timeout});
    }

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @return A command that aligns the robot with the waypoints
     */
    public static Command align(
            List<Pose2d> waypoints,
            int startWaypoint,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts) {
        return align(
                waypoints,
                startWaypoint,
                endWaypoint,
                stopAtEndWaypoint,
                waypointTimeouts,
                KinematicConstraints.DEFAULT);
    }

    private record WaypointData(KinematicState fullStopState, KinematicState velocityState) {}

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param constraints The maximum kinematic constraints to use
     * @return A command that aligns the robot with the waypoints
     */
    public static Command align(
            List<Pose2d> waypoints,
            int startWaypoint,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts,
            KinematicConstraints constraints) {
        RuckigAlignGroup<WaypointData> waypointGroup = new RuckigAlignGroup<WaypointData>(
                constraints.maxVelocity, constraints.maxAcceleration, constraints.maxJerk);
        return align(waypoints, startWaypoint, endWaypoint, stopAtEndWaypoint, waypointTimeouts, waypointGroup)
                .build();
    }

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param waypointGroup The RuckigAlignGroup to use for the alignment (useful for chaining multiple alignments together)
     * @return The RuckigAlignGroup with the waypoints added
     */
    public static RuckigAlignGroup<WaypointData> align(
            List<Pose2d> waypoints,
            int startWaypoint,
            int endWaypoint,
            boolean stopAtEndWaypoint,
            Double[] waypointTimeouts,
            RuckigAlignGroup<WaypointData> waypointGroup) {

        if (waypoints.size() > waypointTimeouts.length) {
            throw new IllegalArgumentException("Waypoints and waypoint timeouts must have the same length");
        }

        if (startWaypoint < 0 || startWaypoint >= waypoints.size()) {
            throw new IllegalArgumentException("startWaypoint is out of bounds");
        }

        if (endWaypoint < 0 || endWaypoint >= waypoints.size()) {
            throw new IllegalArgumentException("endWaypoint is out of bounds");
        }

        for (int i = startWaypoint; i <= endWaypoint; i++) {
            final int index = i;
            waypointGroup.addAlign(
                    () -> {
                        final KinematicState fullStopState = new KinematicState(
                                new double[] {
                                    waypoints.get(index).getX(),
                                    waypoints.get(index).getY(),
                                    waypoints.get(index).getRotation().getRadians()
                                },
                                new double[] {0, 0, 0},
                                new double[] {0, 0, 0});

                        final KinematicState velocityState;
                        if (index != waypoints.size() - 1) {
                            velocityState = getKinematicStateForWaypoint(
                                    index == 0
                                            ? RobotContainer.poseSensorFusion.getEstimatedPosition()
                                            : waypoints.get(index - 1),
                                    waypoints.get(index),
                                    waypoints.get(index + 1),
                                    waypointGroup.getMaxVelocity(),
                                    waypointGroup.getMaxAcceleration(),
                                    true);
                        } else {
                            velocityState = null;
                        }
                        return new WaypointData(fullStopState, velocityState);
                    },
                    data -> {
                        if (index == waypoints.size() - 1 || (stopAtEndWaypoint && index == endWaypoint)) {
                            return new RuckigAlignState(data.fullStopState, AlignMode.Position);
                        } else {
                            return new RuckigAlignState(data.velocityState, AlignMode.Velocity);
                        }
                    },
                    waypointTimeouts[i]);
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
            List<Pose2d> waypoints,
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
                KinematicConstraints.DEFAULT);
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
     * @param constraints The maximum kinematic constraints to use
     * @return
     */
    public static Command alignWithCommand(
            List<Pose2d> waypoints,
            Double[] waypointTimeouts,
            int commandStartWaypoint,
            int commandEndWaypoint,
            Command performCommand,
            KinematicConstraints constraints) {

        if (waypoints.size() > waypointTimeouts.length) {
            throw new IllegalArgumentException("Waypoints and waypoint timeouts must have the same length");
        }

        return Commands.defer(
                () -> {
                    RuckigAlignGroup<WaypointData> waypointGroup = new RuckigAlignGroup<WaypointData>(
                            constraints.maxVelocity, constraints.maxAcceleration, constraints.maxJerk);

                    align(
                            waypoints,
                            Math.min(commandEndWaypoint, getCurrentWaypoint(waypoints) + 1),
                            commandEndWaypoint,
                            false,
                            waypointTimeouts,
                            waypointGroup.newGroup());
                    align(
                            waypoints,
                            commandEndWaypoint + 1,
                            waypoints.size() - 1,
                            true,
                            waypointTimeouts,
                            waypointGroup.newGroup());

                    Boolean[] firstAlignFinished = new Boolean[] {false};

                    return waypointGroup
                            .build(0)
                            .andThen(() -> firstAlignFinished[0] = true)
                            /* if waypoint align finished we can assume we are at the end */
                            .alongWith(new WaitUntilCommand(() -> getCurrentWaypoint(waypoints) == commandStartWaypoint
                                            || firstAlignFinished[0])
                                    .andThen(performCommand))
                            .andThen(waypointGroup.build(1));
                },
                Set.of(RobotContainer.drivetrain));
    }
}
