package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoPath;
import frc.robot.utils.SimpleMath;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.recordrobotics.ruckig.InputParameter3;
import org.recordrobotics.ruckig.OutputParameter3;
import org.recordrobotics.ruckig.Ruckig3;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;
import org.recordrobotics.ruckig.enums.DurationDiscretization;
import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.enums.Synchronization;

public class RuckigAlign extends Command {

    /**
     * Allows sequential RuckigAlign commands to not reset the trajectory state on every initialize.
     * Useful for following waypoints with velocity mode.
     */
    public static class RuckigAlignGroup<T> {

        private record AlignEntry<T>(Supplier<T> initializer, Function<T, RuckigAlignState> state, double timeout) {}

        private final List<AlignEntry<T>> states = new ArrayList<>();
        private final List<Integer> groups = new ArrayList<>();
        private final double[] maxVelocity;
        private final double[] maxAcceleration;
        private final double[] maxJerk;

        /**
         * Create a RuckigAlignGroup with the given constraints
         * @param maxVelocity the max velocity for each of the 3 dimensions (x, y, rotation)
         * @param maxAcceleration the max acceleration for each of the 3 dimensions (x, y, rotation)
         * @param maxJerk the max jerk for each of the 3 dimensions (x, y, rotation)
         */
        public RuckigAlignGroup(double[] maxVelocity, double[] maxAcceleration, double[] maxJerk) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
        }

        /**
         * Start a new group of align states (used for splitting the group command into multiple)
         * @return this (for chaining)
         */
        public RuckigAlignGroup<T> newGroup() {
            groups.add(states.size());
            return this;
        }

        /**
         * Add an align state to the group
         * @param initializer Function to initialize any parameters needed for the state
         * @param state Function to get the RuckigAlignState from the parameters
         * @param timeout Timeout for this align state
         * @return this (for chaining)
         */
        public RuckigAlignGroup<T> addAlign(
                Supplier<T> initializer, Function<T, RuckigAlignState> state, double timeout) {
            states.add(new AlignEntry<>(initializer, state, timeout));
            return this;
        }

        public double[] getMaxVelocity() {
            return maxVelocity;
        }

        public double[] getMaxAcceleration() {
            return maxAcceleration;
        }

        public double[] getMaxJerk() {
            return maxJerk;
        }

        /**
         * Build the RuckigAlign command sequence for all groups
         * @return the command sequence
         */
        public Command build() {
            return build(0, groups.size() - 1);
        }

        /**
         * Build the RuckigAlign command sequence for a specific group
         * @param group the group index
         * @return the command sequence
         */
        public Command build(int group) {
            return build(group, group);
        }

        /**
         * Build the RuckigAlign command sequence for a range of groups
         * @param startGroup the starting group index (inclusive)
         * @param endGroup the ending group index (inclusive)
         * @return the command sequence
         */
        public Command build(int startGroup, int endGroup) {
            if (states.isEmpty()) return Commands.none();

            final int startIndex;
            final int endIndex;

            if (groups.isEmpty()) {
                // If no groups were defined, treat all states as a single group
                startIndex = 0;
                endIndex = states.size();
            } else {
                if (startGroup < 0
                        || startGroup >= groups.size()
                        || endGroup < 0
                        || endGroup >= groups.size()
                        || startGroup > endGroup) {
                    throw new IllegalArgumentException("Invalid group indices");
                }

                startIndex = groups.get(startGroup);
                endIndex = (endGroup + 1 < groups.size()) ? groups.get(endGroup + 1) : states.size();
            }

            List<Command> commands = new ArrayList<>();
            for (int i = startIndex; i < endIndex; i++) {
                final int index = i;
                final AlignEntry<T> entry = states.get(index);
                commands.add(Commands.defer(
                                () -> {
                                    final T param = entry.initializer().get();
                                    return new RuckigAlign(
                                            () -> entry.state().apply(param),
                                            maxVelocity,
                                            maxAcceleration,
                                            maxJerk,
                                            index == 0);
                                },
                                Set.of(RobotContainer.drivetrain))
                        .withTimeout(entry.timeout()));
            }
            return Commands.sequence(commands.toArray(Command[]::new));
        }
    }

    private static final Ruckig3 ruckig = new Ruckig3();
    private static final InputParameter3 input = new InputParameter3();
    private static final OutputParameter3 output = new OutputParameter3();

    private static final PIDController xPid = new PIDController(5, 0, 0.03);
    private static final PIDController yPid = new PIDController(5, 0, 0.03);
    private static final PIDController rPid = new PIDController(7, 0, 0.04);

    private static AlignMode currentMode = AlignMode.POSITION;

    public enum AlignMode {
        POSITION, // Full stop at target
        VELOCITY // Keep moving at target velocity
    }

    public record RuckigAlignState(KinematicState kinematicState, AlignMode alignMode) {}

    static {
        input.setDurationDiscretization(DurationDiscretization.Discrete);
        input.setDefaultSynchronization(Synchronization.Phase);
        input.setPerDoFSynchronization(
                new Synchronization[] {Synchronization.Phase, Synchronization.Phase, Synchronization.None});
        setPositionModeTolerance();
        rPid.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final Supplier<RuckigAlignState> targetStateSupplier;
    private final double[] maxVelocity;
    private final double[] maxAcceleration;
    private final double[] maxJerk;
    private final boolean resetTrajectory;

    private Result result;

    private static boolean lastAlignSuccessful = false;

    public RuckigAlign(
            Supplier<RuckigAlignState> targetStateSupplier,
            double[] maxVelocity,
            double[] maxAcceleration,
            double[] maxJerk) {
        this(targetStateSupplier, maxVelocity, maxAcceleration, maxJerk, true);
    }

    private RuckigAlign(
            Supplier<RuckigAlignState> targetStateSupplier,
            double[] maxVelocity,
            double[] maxAcceleration,
            double[] maxJerk,
            boolean resetTrajectory) {
        this.targetStateSupplier = targetStateSupplier;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        this.resetTrajectory = resetTrajectory;

        addRequirements(RobotContainer.drivetrain);
    }

    private static double[] pose2dToArray(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            SimpleMath.closestTarget(
                    input.getCurrentPosition()[2],
                    SimpleMath.normalizeAngle(pose.getRotation().getRadians()))
        };
    }

    private static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    private static void setTargetState(KinematicState state) {
        double[] targetPosition = state.position();
        targetPosition[2] =
                SimpleMath.closestTarget(input.getCurrentPosition()[2], SimpleMath.normalizeAngle(targetPosition[2]));
        input.setTargetPosition(targetPosition);
        input.setTargetVelocity(state.velocity());
        input.setTargetAcceleration(state.acceleration());
    }

    private static void applyAlignState(RuckigAlignState state) {
        setTargetState(state.kinematicState());
        if (state.alignMode() == AlignMode.POSITION) {
            setPositionModeTolerance();
        } else {
            setVelocityModeTolerance();
        }
    }

    private void reset() {
        Pose2d pose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        input.setCurrentPosition(pose2dToArray(pose));
        input.setCurrentVelocity(chassisSpeedsToArray(ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotContainer.drivetrain.getChassisSpeeds(), pose.getRotation())));
        input.setCurrentAcceleration(chassisSpeedsToArray(ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotContainer.drivetrain.getChassisAcceleration(), pose.getRotation())));
        xPid.reset();
        yPid.reset();
        rPid.reset();
        result = Result.Working;
    }

    /**
     * Tight tolerance for position mode (final full-stop target)
     */
    private static void setPositionModeTolerance() {
        xPid.setTolerance(Constants.Align.TRANSLATIONAL_TOLERANCE, Constants.Align.TRANSLATIONAL_VELOCITY_TOLERANCE);
        yPid.setTolerance(Constants.Align.TRANSLATIONAL_TOLERANCE, Constants.Align.TRANSLATIONAL_VELOCITY_TOLERANCE);
        rPid.setTolerance(Constants.Align.ROTATIONAL_TOLERANCE, Constants.Align.ROTATIONAL_VELOCITY_TOLERANCE);
        currentMode = AlignMode.POSITION;
    }

    private static final double VELOCITY_TOLERANCE_MULTIPLIER = 5.0;

    /**
     * Increases the tolerance for velocity mode
     * to insure smooth waypoint following (final target is still in position mode)
     */
    private static void setVelocityModeTolerance() {
        xPid.setTolerance(
                Constants.Align.TRANSLATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, Double.POSITIVE_INFINITY);
        yPid.setTolerance(
                Constants.Align.TRANSLATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, Double.POSITIVE_INFINITY);
        rPid.setTolerance(
                Constants.Align.ROTATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, Double.POSITIVE_INFINITY);
        currentMode = AlignMode.VELOCITY;
    }

    @Override
    public void initialize() {
        input.setMaxVelocity(maxVelocity);
        input.setMaxAcceleration(maxAcceleration);
        input.setMaxJerk(maxJerk);

        if (resetTrajectory) {
            reset();
        }

        applyAlignState(targetStateSupplier.get());

        setLastAlignSuccessful(false);
    }

    private static double feedforward(double velocity, double acceleration, double jerk) {
        final double kV = 1.0;
        final double kA = 0.1;
        final double kJ = 0.01;
        return kV * velocity + kA * acceleration + kJ * jerk;
    }

    @Override
    public void execute() {
        applyAlignState(targetStateSupplier.get());

        result = ruckig.update(input, output);

        Pose2d currentPose = RobotContainer.poseSensorFusion.getEstimatedPosition();

        double[] newPosition = output.getNewPosition();
        double[] newVelocity = output.getNewVelocity();
        double[] newAcceleration = output.getNewAcceleration();
        double[] newJerk = output.getNewJerk();

        Logger.recordOutput(
                "Ruckig/Target",
                new Pose2d(
                        input.getTargetPosition()[0],
                        input.getTargetPosition()[1],
                        new Rotation2d(input.getTargetPosition()[2])));

        Logger.recordOutput(
                "Ruckig/Setpoint", new Pose2d(newPosition[0], newPosition[1], new Rotation2d(newPosition[2])));
        Logger.recordOutput(
                "Ruckig/SetpointVelocity",
                new Transform2d(newVelocity[0], newVelocity[1], new Rotation2d(newVelocity[2])));
        Logger.recordOutput(
                "Ruckig/SetpointAcceleration",
                new Transform2d(newAcceleration[0], newAcceleration[1], new Rotation2d(newAcceleration[2])));
        Logger.recordOutput("Ruckig/SetpointJerk", new Transform2d(newJerk[0], newJerk[1], new Rotation2d(newJerk[2])));

        // Calculate the new velocities using PID and velocity feedforward
        double vx = xPid.calculate(currentPose.getX(), newPosition[0])
                + feedforward(newVelocity[0], newAcceleration[0], newJerk[0]);
        double vy = yPid.calculate(currentPose.getY(), newPosition[1])
                + feedforward(newVelocity[1], newAcceleration[1], newJerk[1]);
        double vr = rPid.calculate(currentPose.getRotation().getRadians(), newPosition[2])
                + feedforward(newVelocity[2], newAcceleration[2], newJerk[2]);

        AutoPath.CONTROL_MODIFER.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, vr), currentPose.getRotation()));

        output.passToInput(input);

        double ex = Math.abs(currentPose.getX() - newPosition[0]);
        double ey = Math.abs(currentPose.getY() - newPosition[1]);
        double er = Math.abs(SimpleMath.closestTarget(
                        newPosition[2],
                        SimpleMath.normalizeAngle(currentPose.getRotation().getRadians()))
                - newPosition[2]);

        Logger.recordOutput("Ruckig/Errors", new double[] {ex, ey, er});

        // If the error is too large, reset the trajectory to current position for more accurate motion
        if (ex * ex + ey * ey > 0.5 || Math.abs(er) > 1.0) {
            reset();
        }
    }

    private static final double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD = 0.05; // meters

    @Override
    public boolean isFinished() {
        boolean finished = false;

        if (currentMode == AlignMode.POSITION) {
            finished = result != Result.Working && xPid.atSetpoint() && yPid.atSetpoint() && rPid.atSetpoint();
        } else if (currentMode == AlignMode.VELOCITY) {
            Pose2d currentPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
            double distanceToTarget = Math.hypot(
                    currentPose.getX() - input.getTargetPosition()[0],
                    currentPose.getY() - input.getTargetPosition()[1]);
            finished = distanceToTarget < VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD || result != Result.Working;
        }

        if (finished) {
            setLastAlignSuccessful(true);
        }
        return finished;
    }

    public static boolean lastAlignSuccessful() {
        return lastAlignSuccessful;
    }

    private static void setLastAlignSuccessful(boolean value) {
        lastAlignSuccessful = value;
    }
}
