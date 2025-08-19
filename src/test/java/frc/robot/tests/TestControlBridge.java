package frc.robot.tests;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.OverviewLayout;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;
import java.util.HashMap;
import java.util.Map;

public class TestControlBridge extends AbstractControl {

    static {
        OverviewLayout.setTestControl(getInstance());
    }

    private ReefLevelSwitchValue reefswitch = ReefLevelSwitchValue.L4;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    private Map<Button, Integer> buttonStates = new HashMap<>();
    private Map<Axis, Double> axisStates = new HashMap<>();

    private static TestControlBridge instance;

    public static TestControlBridge getInstance() {
        if (instance == null) {
            instance = new TestControlBridge();
        }
        return instance;
    }

    private boolean getButton(Button btn) {
        int state = buttonStates.getOrDefault(btn, 0);
        // state > 0 is being pressed, state < 0 is being held down, state == 0 is released
        return state != 0;
    }

    @Override
    public void update() {
        for (Map.Entry<Button, Integer> entry : buttonStates.entrySet()) {
            Button btn = entry.getKey();
            int state = entry.getValue();

            if (state > 0) {
                buttonStates.put(btn, state - 1); // Decrement the button state
            } else if (state < 0) {
                buttonStates.put(btn, -1); // Button is held down
            }
        }

        Pair<Double, Double> xy =
                getXY(!(getCoralIntakeRelativeDrive() || getElevatorRelativeDrive() || getClimbRelativeDrive()));

        double x = xy.getFirst() * getDirectionalSpeedLevel();
        double y = xy.getSecond() * getDirectionalSpeedLevel();

        if (getCoralIntakeRelativeDrive()) {
            y = -y;
        } else if (getElevatorRelativeDrive()) {
            double temp = y;
            y = -x;
            x = -temp;
        } else if (getClimbRelativeDrive()) {
            double temp = y;
            y = x;
            x = temp;
        }

        velocity = new Transform2d(x, y, new Rotation2d(getSpin() * getSpinSpeedLevel()));
        acceleration = new Transform2d(
                        velocity.getTranslation()
                                .minus(lastVelocity.getTranslation())
                                .div(0.02),
                        velocity.getRotation().minus(lastVelocity.getRotation()))
                .div(0.02);
        jerk = new Transform2d(
                        acceleration
                                .getTranslation()
                                .minus(lastAcceleration.getTranslation())
                                .div(0.02),
                        acceleration.getRotation().minus(lastAcceleration.getRotation()))
                .div(0.02);

        lastVelocity = velocity;
        lastAcceleration = acceleration;
    }

    @Override
    public DrivetrainControl getDrivetrainControl() {
        if (getElevatorRelativeDrive() || getCoralIntakeRelativeDrive() || getClimbRelativeDrive()) {
            return DrivetrainControl.createRobotRelative(velocity, acceleration, jerk);
        } else {
            return DrivetrainControl.createFieldRelative(
                    velocity,
                    acceleration,
                    jerk,
                    RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());
        }
    }

    @Override
    public Transform2d getRawDriverInput() {
        Pair<Double, Double> xy = getXY(false);
        // Returns the raw driver input as a Transform2d
        return new Transform2d(xy.getFirst(), xy.getSecond(), Rotation2d.fromRadians(getSpin()));
    }

    public Boolean getAutoAlign() {
        return getButton(Button.AUTO_ALIGN);
    }

    public Boolean getAutoAlignNear() {
        return getButton(Button.AUTO_ALIGN_NEAR);
    }

    public Boolean getElevatorRelativeDrive() {
        return getButton(Button.ELEVATOR_RELATIVE_DRIVE)
                || (getAutoScore()
                        && getReefLevelSwitchValue() != ReefLevelSwitchValue.L1); // elevator relative when auto score
    }

    public Boolean getCoralIntakeRelativeDrive() {
        return getButton(Button.CORAL_INTAKE_RELATIVE_DRIVE)
                || (getAutoScore()
                        && getReefLevelSwitchValue() == ReefLevelSwitchValue.L1); // coral relative when auto score
    }

    public Boolean getClimbRelativeDrive() {
        return getButton(Button.CLIMB_RELATIVE_DRIVE);
    }

    public Pair<Double, Double> getXY(boolean orient) {
        double X = SimpleMath.ApplyThresholdAndSensitivity(
                axisStates.getOrDefault(Axis.X, 0.0),
                Constants.Control.JOYSTICK_X_THRESHOLD,
                Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
        double Y = SimpleMath.ApplyThresholdAndSensitivity(
                axisStates.getOrDefault(Axis.Y, 0.0),
                Constants.Control.JOYSTICK_Y_THRESHOLD,
                Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);

        if (orient) return super.OrientXY(new Pair<Double, Double>(X, Y));
        else return new Pair<Double, Double>(X, Y);
    }

    public Double getSpin() {
        // Gets raw twist value
        return SimpleMath.ApplyThresholdAndSensitivity(
                -SimpleMath.Remap(axisStates.getOrDefault(Axis.TWIST, 0.0), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public Boolean getHalfSpeed() {
        return getAutoScore(); // half speed auto enabled when scoring
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.Remap(
                axisStates.getOrDefault(Axis.SPEED_LEVEL, 0.0),
                1,
                -1,
                Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
                Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);

        if (getHalfSpeed()) {
            speed /= 3;
        }

        return speed;
    }

    public Double getSpinSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.Remap(
                axisStates.getOrDefault(Axis.SPEED_LEVEL, 0.0),
                1,
                -1,
                Constants.Control.SPIN_SPEED_METER_LOW,
                Constants.Control.SPIN_SPEED_METER_HIGH);

        if (getHalfSpeed()) {
            speed /= 2;
        }

        return speed;
    }

    @Override
    public Boolean getPoseReset() {
        return getButton(Button.POSE_RESET);
    }

    @Override
    public Boolean getLimelightReset() {
        return getButton(Button.LIMELIGHT_RESET);
    }

    @Override
    public Boolean getKill() {
        return getButton(Button.KILL);
    }

    @Override
    public void vibrate(RumbleType type, double value) {} // no vibrate

    @Override
    public Boolean getAutoScore() {
        return getButton(Button.AUTO_SCORE);
    }

    @Override
    public Boolean getElevatorL2() {
        return getButton(Button.L2);
    }

    @Override
    public Boolean getElevatorL3() {
        return getButton(Button.L3);
    }

    @Override
    public Boolean getElevatorL4() {
        return getButton(Button.L4);
    }

    @Override
    public Boolean getCoralShoot() {
        return getButton(Button.CORAL_SHOOT);
    }

    @Override
    public Boolean getCoralGroundIntake() {
        return getButton(Button.GROUND_INTAKE);
    }

    @Override
    public Boolean getCoralGroundIntakeSimple() {
        return getButton(Button.GROUND_INTAKE_SIMPLE);
    }

    @Override
    public Boolean getReefAlgaeSimple() {
        return getButton(Button.REEF_ALGAE_SIMPLE);
    }

    @Override
    public Boolean getCoralSourceIntake() {
        return getButton(Button.SOURCE_INTAKE);
    }

    @Override
    public Boolean getElevatorAlgaeLow() {
        return getButton(Button.ALGAE_LOW);
    }

    @Override
    public Boolean getElevatorAlgaeHigh() {
        return getButton(Button.ALGAE_HIGH);
    }

    @Override
    public ReefLevelSwitchValue getReefLevelSwitchValue() {
        return reefswitch;
    }

    @Override
    public Boolean getManualOverride() {
        return getButton(Button.MANUAL_OVERRIDE);
    }

    @Override
    public Boolean getGroundAlgae() {
        return getButton(Button.GROUND_ALGAE);
    }

    @Override
    public Boolean getScoreAlgae() {
        return getButton(Button.SCORE_ALGAE);
    }

    @Override
    public Boolean getCoralIntakeScoreL1() {
        return getButton(Button.INTAKE_SCORE_L1);
    }

    @Override
    public LinearVelocity getManualElevatorVelocity() {
        double axis = SimpleMath.povToVector((int) Math.round(axisStates.getOrDefault(Axis.POV, -1.0)))
                .getY();
        return Centimeters.of(50).per(Seconds).times(axis);
    }

    @Override
    public AngularVelocity getManualElevatorArmVelocity() {
        double axis = SimpleMath.povToVector((int) Math.round(axisStates.getOrDefault(Axis.POV, -1.0)))
                .getX();
        return Degrees.of(180).per(Seconds).times(axis);
    }

    @Override
    public Boolean getClimb() {
        return getButton(Button.CLIMB);
    }

    @Override
    public Boolean getCoralSourceIntakeAuto() {
        Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
        SourcePosition closestSource = IGamePosition.closestTo(robot, SourcePosition.values());

        boolean nearSource = closestSource.getPose().getTranslation().getDistance(robot.getTranslation()) < 2.3
                && Math.abs(closestSource
                                .getPose()
                                .getRotation()
                                .minus(robot.getRotation())
                                .getMeasure()
                                .abs(Degrees))
                        < 80;

        return nearSource;
    }

    /**
     * Sets the reef level switch value.
     * @param level the reef level switch value to set
     */
    public void setReefLevel(ReefLevelSwitchValue level) {
        this.reefswitch = level;
    }

    /**
     * Holds a button indefinitely until released {@link #releaseButton(Button)}.
     * @param btn the button to hold
     */
    public void holdButton(Button btn) {
        buttonStates.put(btn, -1);
    }

    /**
     * Releases a button.
     * @param btn the button to release
     */
    public void releaseButton(Button btn) {
        buttonStates.put(btn, 0);
    }

    /**
     * Presses a button for a certain number of cycles.
     * The button will be pressed for the specified number of cycles, after which it will be released.
     * @param btn the button to press
     * @param cycles the number of cycles to press the button for
     */
    public void pressButton(Button btn, int cycles) {
        // add 1 to cycle count because of update order - control periodic (where cycles is decremented) is called
        // before trigger periodic where value is read
        buttonStates.put(btn, cycles + 1);
    }

    /**
     * Sets the value of an axis.
     * @param axis the axis to set
     * @param value the value to set the axis to
     */
    public void setAxis(Axis axis, double value) {
        axisStates.put(axis, value);
    }

    /**
     * Resets the control bridge state.
     * This method resets all button states, axis states, and motion states to their initial values.
     */
    public void reset() {
        lastVelocity = new Transform2d();
        lastAcceleration = new Transform2d();
        velocity = new Transform2d();
        acceleration = new Transform2d();
        jerk = new Transform2d();

        buttonStates.clear();
        axisStates.clear();
        reefswitch = ReefLevelSwitchValue.L4;
    }

    public enum Button {
        AUTO_ALIGN,
        AUTO_ALIGN_NEAR,
        ELEVATOR_RELATIVE_DRIVE,
        CORAL_INTAKE_RELATIVE_DRIVE,
        CLIMB_RELATIVE_DRIVE,
        POSE_RESET,
        LIMELIGHT_RESET,
        KILL,
        AUTO_SCORE,
        L2,
        L3,
        L4,
        CORAL_SHOOT,
        GROUND_INTAKE,
        GROUND_INTAKE_SIMPLE,
        REEF_ALGAE_SIMPLE,
        SOURCE_INTAKE,
        ALGAE_LOW,
        ALGAE_HIGH,
        MANUAL_OVERRIDE,
        GROUND_ALGAE,
        SCORE_ALGAE,
        INTAKE_SCORE_L1,
        CLIMB
    }

    public enum Axis {
        X,
        Y,
        TWIST,
        SPEED_LEVEL,
        POV
    }
}
