package frc.robot.utils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;
import org.littletonrobotics.junction.Logger;

/**
 * A collection of trapezoidal position constraints defining the relationship between a single input axis and multiple output axes.
 * <p>For example, this can be used to define a mapping from elevator height to elevator arm position.</p>
 */
public class TrapezoidalPositionConstraint {

    private final TreeSet<Constraint> constraints =
            new TreeSet<>(Comparator.comparingDouble((Constraint c) -> c.start).thenComparingDouble(c -> c.end));

    private final Axis inputAxis;
    private final List<Axis> outputAxes;

    /**
     * Creates a new TrapezoidalPositionConstraint with the specified input axis.
     * @param inputAxis the input axis constraints
     */
    public TrapezoidalPositionConstraint(AxisConstraints inputAxis) {
        this.inputAxis = new Axis(inputAxis);
        this.outputAxes = new ArrayList<>();
    }

    /**
     * Adds a new output axis.
     * @param axisConstraints the axis constraints
     * @return this (for chaining)
     */
    public TrapezoidalPositionConstraint addOutputAxis(AxisConstraints axisConstraints) {
        outputAxes.add(new Axis(axisConstraints));
        return this;
    }

    /**
     * Gets the input axis.
     * @return the input axis
     */
    public Axis getInputAxis() {
        return inputAxis;
    }

    /**
     * Gets the number of output axes.
     * @return the number of output axes
     */
    public int getOutputAxesCount() {
        return outputAxes.size();
    }

    /**
     * Gets the output axis at the specified index.
     * @param index the index of the output axis
     * @return the output axis at the specified index
     */
    public Axis getOutputAxis(int index) {
        return outputAxes.get(index);
    }

    /**
     * Adds a new constraint. If the new constraint overlaps with existing enabled constraints, an exception is thrown and the constraint is not added.
     * @param constraint the constraint to add
     * @return this (for chaining)
     */
    public TrapezoidalPositionConstraint addConstraint(Constraint constraint) {
        constraints.add(constraint);

        if (hasOverlappingConstraints()) {
            constraints.remove(constraint);
            throw new IllegalArgumentException("Added constraint overlaps with existing enabled constraints.");
        } else if (constraint.outputAxisIndex < 0 || constraint.outputAxisIndex >= getOutputAxesCount()) {
            constraints.remove(constraint);
            throw new IllegalArgumentException("Output axis index out of bounds.");
        }

        constraint.owner = this;

        return this;
    }

    /**
     * Removes a constraint.
     * @param constraint the constraint to remove
     * @return this (for chaining)
     */
    public TrapezoidalPositionConstraint removeConstraint(Constraint constraint) {
        constraints.remove(constraint);
        constraint.owner = null;

        return this;
    }

    /**
     * Checks if there are any overlapping enabled constraints.
     * @return true if there are overlapping enabled constraints, false otherwise.
     */
    private boolean hasOverlappingConstraints() {
        for (int axis = 0; axis < getOutputAxesCount(); axis++) {
            Constraint prev = null;
            for (Constraint constraint : constraints) {
                if (!constraint.enabled || constraint.outputAxisIndex != axis) {
                    continue;
                }

                if (prev != null && constraint.start < prev.end) {
                    return true;
                }

                prev = constraint;
            }
        }

        return false;
    }

    /**
     * Finds the closest applicable constraint for the given input position and velocity on the specified output axis.
     * <p>If the input position is within a constraint's range, that constraint is returned immediately.</p>
     * @param inputPosition The current position of the input axis.
     * @param inputVelocity The current velocity of the input axis.
     * @param axisIndex The index of the output axis to consider.
     * @return The closest applicable constraint, or null if none found.
     */
    private Constraint getClosestConstraint(double inputPosition, double inputVelocity, int axisIndex) {
        final double ALLOWED_INPUT_TOLERANCE = 0.03;
        Constraint bestConstraint = null;
        double bestDistance = Double.MAX_VALUE;

        // Since constraints are sorted from lowest to highest start position, we can optimize our search
        Iterator<Constraint> iterator = inputVelocity >= 0 ? constraints.iterator() : constraints.descendingIterator();
        while (iterator.hasNext()) {
            Constraint constraint = iterator.next();

            if (!constraint.enabled || constraint.outputAxisIndex != axisIndex) {
                continue;
            }

            double distance = Double.MAX_VALUE;

            // Check if we are within the constraint range
            if (constraint.isInputWithinConstraint(inputPosition, ALLOWED_INPUT_TOLERANCE, ALLOWED_INPUT_TOLERANCE)) {
                // We are inside the constraint
                return constraint;
            } else if (inputVelocity > 0 && inputPosition < constraint.start) {
                // Only consider constraints ahead of velocity
                distance = constraint.start - inputPosition;
            } else if (inputVelocity < 0 && inputPosition > constraint.end) {
                // Only consider constraints ahead of velocity
                distance = inputPosition - constraint.end;
            }

            if (distance < bestDistance) {
                bestDistance = distance;
                bestConstraint = constraint;
            }
        }

        return bestConstraint;
    }

    private void setAxisSetpointIfCloser(Axis axis, double newSetpoint) {
        double currentDistance = Math.abs(axis.setpoint - axis.currentState.position);
        double newDistance = Math.abs(newSetpoint - axis.currentState.position);

        if (newDistance < currentDistance) {
            axis.setpoint = newSetpoint;
        }
    }

    /**
     * Calculates and updates the output axes setpoints based on the current input axis position and the defined constraints.
     * @return true if any constraints were applied, false otherwise
     */
    public boolean calculate() {
        final double ALLOWED_INPUT_TOLERANCE = 0.03;
        final double ALLOWED_OUTPUT_TOLERANCE = 0.1;

        boolean constraintApplied = false;

        double inputPosition = inputAxis.currentState.position;
        double inputVelocity = inputAxis.currentState.velocity;

        // Assume no constraints at first
        inputAxis.setpoint = inputAxis.target;

        for (int axisIndex = 0; axisIndex < outputAxes.size(); axisIndex++) {
            Axis outputAxis = outputAxes.get(axisIndex);

            // Assume no constraints at first
            outputAxis.setpoint = outputAxis.target;

            Constraint constraint = getClosestConstraint(inputPosition, inputVelocity, axisIndex);

            Logger.recordOutput(
                    "TrapezoidalPositionConstraint/Axis" + axisIndex + "/AppliedConstraint",
                    constraint != null ? constraint.start + " to " + constraint.end : "None");

            // Apply the best constraint if found and the output axis target is outside the constraint range
            if (constraint != null
                    && !constraint.isOututWithinConstraint(
                            outputAxis.target, ALLOWED_OUTPUT_TOLERANCE, ALLOWED_OUTPUT_TOLERANCE)) {
                Logger.recordOutput("TrapezoidalPositionConstraint/Axis" + axisIndex + "/AxisWithinConstraint", false);
                double timeToReachInputMin = SimpleMath.timeToTargetTrapezoidal(
                        outputAxis.currentState.position,
                        constraint.minPosition,
                        outputAxis.currentState.velocity,
                        outputAxis.constraints.maxVelocity,
                        outputAxis.constraints.maxAcceleration);

                double timeToReachInputMax = SimpleMath.timeToTargetTrapezoidal(
                        outputAxis.currentState.position,
                        constraint.maxPosition,
                        outputAxis.currentState.velocity,
                        outputAxis.constraints.maxVelocity,
                        outputAxis.constraints.maxAcceleration);

                double closestPosition =
                        timeToReachInputMin < timeToReachInputMax ? constraint.minPosition : constraint.maxPosition;
                double timeToReachClosestPosition = Math.min(timeToReachInputMin, timeToReachInputMax);

                double inputPositionAtThatTime = inputPosition + inputVelocity * timeToReachClosestPosition;

                Logger.recordOutput(
                        "TrapezoidalPositionConstraint/Axis" + axisIndex + "/InputPositionAtThatTime",
                        inputPositionAtThatTime);

                double distanceToConstraintStart = Math.abs(inputPosition - constraint.start);
                double distanceToConstraintEnd = Math.abs(inputPosition - constraint.end);
                double closestDistanceToConstraint = Math.min(distanceToConstraintStart, distanceToConstraintEnd);
                double closestConstraintPosition =
                        distanceToConstraintStart < distanceToConstraintEnd ? constraint.start : constraint.end;

                if (closestDistanceToConstraint <= ALLOWED_INPUT_TOLERANCE) {
                    // If we start moving output to within constraint, we will reach it right as we enter the constraint
                    // Set input to its target - move normally
                    setAxisSetpointIfCloser(inputAxis, inputAxis.target);
                    // Set output to the closest position within the constraint
                    outputAxis.setpoint = closestPosition;

                    constraintApplied = true;
                } else if (inputVelocity >= 0
                        ? inputPositionAtThatTime > constraint.start
                        : inputPositionAtThatTime < constraint.end) {
                    // If we start moving output to within constraint, we will reach it after we already enter the
                    // constraint
                    // Set input to the constraint edge so we don't overshoot
                    setAxisSetpointIfCloser(inputAxis, closestConstraintPosition);
                    // Set output to the closest position within the constraint
                    outputAxis.setpoint = closestPosition;
                    // Once output axis is within constraint, the outer if will restore the original input axis setpoint

                    constraintApplied = true;
                }
            } else {
                Logger.recordOutput("TrapezoidalPositionConstraint/Axis" + axisIndex + "/AxisWithinConstraint", true);
            }
        }

        return constraintApplied;
    }

    /**
     * A trapezoidal position constraint defining a mapping from an input position range of one mechanism to an output position range of another.
     * <p>For example, this can be used to define a mapping from elevator height to elevator arm position.</p>
     */
    public static class Constraint {

        private double start;
        private double end;
        private double minPosition;
        private double maxPosition;

        private int outputAxisIndex;

        private boolean enabled;

        private TrapezoidalPositionConstraint owner;

        private boolean wasInputWithinConstraint = false;
        private boolean wasOutputWithinConstraint = false;

        /**
         * Creates a new constraint.
         * @param inputPosition1 The first input position defining the input range.
         * @param inputPosition2 The second input position defining the input range.
         * @param outputPosition1 The first output position defining the output range.
         * @param outputPosition2 The second output position defining the output range.
         * @param outputAxisIndex The index of the output axis this constraint applies to.
         */
        public Constraint(
                double inputPosition1,
                double inputPosition2,
                double outputPosition1,
                double outputPosition2,
                int outputAxisIndex) {
            this(
                    Math.min(inputPosition1, inputPosition2),
                    Math.max(inputPosition1, inputPosition2),
                    Math.min(outputPosition1, outputPosition2),
                    Math.max(outputPosition1, outputPosition2),
                    outputAxisIndex,
                    true);
        }

        /**
         * Creates a new constraint with ordered parameters.
         * @param start The start of the input position range.
         * @param end The end of the input position range.
         * @param minPosition The minimum output position.
         * @param maxPosition The maximum output position.
         * @param outputAxisIndex The index of the output axis this constraint applies to.
         * @param enabled Whether this constraint is enabled.
         */
        private Constraint(
                double start,
                double end,
                double minPosition,
                double maxPosition,
                int outputAxisIndex,
                boolean enabled) {
            this.start = start;
            this.end = end;
            this.minPosition = minPosition;
            this.maxPosition = maxPosition;
            this.outputAxisIndex = outputAxisIndex;
            this.enabled = enabled;
        }

        /**
         * Returns whether this constraint is enabled.
         * @return true if enabled, false otherwise
         */
        public boolean isEnabled() {
            return enabled;
        }

        /**
         * Sets whether this constraint is enabled. If enabling this constraint would cause overlap with existing enabled constraints, an exception is thrown and the state is not changed.
         * @param enabled true to enable, false to disable
         * @return this (for chaining)
         */
        public Constraint setEnabled(boolean enabled) {
            this.enabled = enabled;
            if (owner != null && owner.hasOverlappingConstraints()) {
                this.enabled = !enabled; // revert
                throw new IllegalArgumentException(
                        "Setting this constraint enabled causes overlap with existing enabled constraints.");
            }
            return this;
        }

        /**
         * Gets the start of the input position range.
         * @return the start of the input position range
         */
        public double getStart() {
            return start;
        }

        /**
         * Gets the end of the input position range.
         * @return the end of the input position range
         */
        public double getEnd() {
            return end;
        }

        /**
         * Gets the minimum output position.
         * @return the minimum output position
         */
        public double getMinPosition() {
            return minPosition;
        }

        /**
         * Gets the maximum output position.
         * @return the maximum output position
         */
        public double getMaxPosition() {
            return maxPosition;
        }

        /**
         * Gets the output axis index.
         * @return the output axis index
         */
        public int getOutputAxisIndex() {
            return outputAxisIndex;
        }

        /**
         * Sets the output axis index.
         * @param outputAxisIndex the output axis index
         * @return this (for chaining)
         */
        public Constraint setOutputAxisIndex(int outputAxisIndex) {
            this.outputAxisIndex = outputAxisIndex;
            if (owner != null && (outputAxisIndex < 0 || outputAxisIndex >= owner.getOutputAxesCount())) {
                throw new IllegalArgumentException("Output axis index out of bounds.");
            }
            return this;
        }

        public boolean isInputWithinConstraint(double position, double enterTolerance, double exitTolerance) {
            if (wasInputWithinConstraint) {
                if (position <= start - exitTolerance || position >= end + exitTolerance) {
                    wasInputWithinConstraint = false;
                    return false;
                } else {
                    return true;
                }
            } else {
                if (position >= start + enterTolerance && position <= end - enterTolerance) {
                    wasInputWithinConstraint = true;
                    return true;
                } else {
                    return false;
                }
            }
        }

        public boolean isOututWithinConstraint(double position, double enterTolerance, double exitTolerance) {
            if (wasOutputWithinConstraint) {
                if (position <= minPosition - exitTolerance || position >= maxPosition + exitTolerance) {
                    wasOutputWithinConstraint = false;
                    return false;
                } else {
                    return true;
                }
            } else {
                if (position >= minPosition + enterTolerance && position <= maxPosition - enterTolerance) {
                    wasOutputWithinConstraint = true;
                    return true;
                } else {
                    return false;
                }
            }
        }
    }

    /**
     * Constraints for an axis, including maximum velocity and acceleration.
     */
    public record AxisConstraints(double maxVelocity, double maxAcceleration) {}
    /**
     * Current state of an axis, including position and velocity.
     */
    public record AxisState(double position, double velocity) {}

    /**
     * An axis with target, setpoint, constraints, and current state.
     */
    public static class Axis {

        private double target;
        private double setpoint;
        private AxisConstraints constraints;
        private AxisState currentState;

        /**
         * Creates a new axis with the specified constraints.
         * @param constraints the axis constraints
         */
        public Axis(AxisConstraints constraints) {
            this(0.0, 0.0, constraints, new AxisState(0.0, 0.0));
        }

        /**
         * Creates a new axis with the specified parameters.
         * @param target The target position.
         * @param setpoint The current setpoint position.
         * @param constraints The axis constraints.
         * @param currentState The current state of the axis.
         */
        private Axis(double target, double setpoint, AxisConstraints constraints, AxisState currentState) {
            this.target = target;
            this.setpoint = setpoint;
            this.constraints = constraints;
            this.currentState = currentState;
        }

        /**
         * Gets the target position.
         * @return the target position
         */
        public double getTarget() {
            return target;
        }

        /**
         * Gets the current setpoint position.
         * @return the current setpoint position
         */
        public double getSetpoint() {
            return setpoint;
        }

        /**
         * Sets the current target position.
         * @param target the current target position
         */
        public void setTarget(double target) {
            this.target = target;
        }

        /**
         * Gets the axis constraints.
         * @return the axis constraints
         */
        public AxisConstraints getConstraints() {
            return constraints;
        }

        /**
         * Sets the axis constraints.
         * @param constraints the axis constraints
         */
        public void setConstraints(AxisConstraints constraints) {
            this.constraints = constraints;
        }

        /**
         * Gets the current state of the axis.
         * @return the current state of the axis
         */
        public AxisState getCurrentState() {
            return currentState;
        }

        /**
         * Sets the current state of the axis.
         * @param currentState the current state of the axis
         */
        public void setCurrentState(AxisState currentState) {
            this.currentState = currentState;
        }
    }
}
