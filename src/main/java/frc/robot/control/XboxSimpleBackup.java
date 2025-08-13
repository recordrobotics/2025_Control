package frc.robot.control;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

public class XboxSimpleBackup extends AbstractControl {

    private XboxController xbox_controller;

    private ReefLevelSwitchValue reefswitch = ReefLevelSwitchValue.L4;

    public XboxSimpleBackup(int xboxPort) {
        xbox_controller = new XboxController(xboxPort);

        new Trigger(() -> xbox_controller.getAButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L1).ignoringDisable(true));
        new Trigger(() -> xbox_controller.getXButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L2).ignoringDisable(true));
        new Trigger(() -> xbox_controller.getBButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L3).ignoringDisable(true));
        new Trigger(() -> xbox_controller.getYButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L4).ignoringDisable(true));
    }

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    @Override
    public void update() {
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
    public Transform2d getRawDriverInput() {
        Pair<Double, Double> xy = getXY(false);
        // Returns the raw driver input as a Transform2d
        return new Transform2d(xy.getFirst(), xy.getSecond(), Rotation2d.fromRadians(getSpin()));
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

    public Boolean getAutoAlign() {
        return false;
    }

    public Boolean getAutoAlignNear() {
        return false;
    }

    public Boolean getElevatorRelativeDrive() {
        return xbox_controller.getPOV() == 0 // POV up
                || (getAutoScore()
                        && getReefLevelSwitchValue()
                                != ReefLevelSwitchValue.L1); // elevator relative when auto score not L1
    }

    public Boolean getCoralIntakeRelativeDrive() {
        return xbox_controller.getPOV() == 270 // POV left
                || (getAutoScore()
                        && getReefLevelSwitchValue() == ReefLevelSwitchValue.L1); // coral relative when auto score L1
    }

    public Boolean getClimbRelativeDrive() {
        return xbox_controller.getPOV() == 180; // POV down
    }

    public Pair<Double, Double> getXY(boolean orient) {
        double X = SimpleMath.ApplyThresholdAndSensitivity(
                xbox_controller.getLeftX(),
                Constants.Control.JOYSTICK_X_THRESHOLD,
                Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
        double Y = SimpleMath.ApplyThresholdAndSensitivity(
                xbox_controller.getLeftY(),
                Constants.Control.JOYSTICK_Y_THRESHOLD,
                Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);

        if (orient) return super.OrientXY(new Pair<Double, Double>(X, Y));
        else return new Pair<Double, Double>(X, Y);
    }

    public Double getSpin() {
        // Gets raw twist value
        return SimpleMath.ApplyThresholdAndSensitivity(
                -SimpleMath.Remap(xbox_controller.getRightX(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public Boolean getHalfSpeed() {
        return getAutoScore(); // half speed auto enabled when scoring
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = 2;
        // SimpleMath.Remap(
        //     joystick.getRawAxis(3),
        //     1,
        //     -1,
        //     Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
        //     Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);

        if (getHalfSpeed()) {
            speed /= 3;
        }

        return speed;
    }

    public Double getSpinSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = 2;
        // SimpleMath.Remap(
        //     joystick.getRawAxis(3),
        //     1,
        //     -1,
        //     Constants.Control.SPIN_SPEED_METER_LOW,
        //     Constants.Control.SPIN_SPEED_METER_HIGH);

        if (getHalfSpeed()) {
            speed /= 2;
        }

        return speed;
    }

    @Override
    public Boolean getPoseReset() {
        return xbox_controller.getBackButton();
    }

    @Override
    public Boolean getLimelightReset() {
        return xbox_controller.getStartButton();
    }

    @Override
    public Boolean getKill() {
        return xbox_controller.getPOV() == 90;
    }

    @Override
    public void vibrate(RumbleType type, double value) {} // no vibrate on the joystick

    @Override
    public Boolean getAutoScore() {
        return xbox_controller.getRightBumperButton();
    }

    @Override
    public Boolean getElevatorL2() {
        return false;
    }

    @Override
    public Boolean getElevatorL3() {
        return false;
    }

    @Override
    public Boolean getElevatorL4() {
        return false;
    }

    @Override
    public Boolean getCoralShoot() {
        return false;
    }

    @Override
    public Boolean getCoralGroundIntake() {
        return false;
    }

    @Override
    public Boolean getCoralGroundIntakeSimple() {
        return xbox_controller.getRightTriggerAxis() > 0.3;
    }

    @Override
    public Boolean getReefAlgaeSimple() {
        return xbox_controller.getLeftBumperButton();
    }

    @Override
    public Boolean getCoralSourceIntake() {
        return false;
    }

    @Override
    public Boolean getElevatorAlgaeLow() {
        return false;
    }

    @Override
    public Boolean getElevatorAlgaeHigh() {
        return false;
    }

    @Override
    public ReefLevelSwitchValue getReefLevelSwitchValue() {
        return reefswitch;
    }

    @Override
    public Boolean getManualOverride() {
        return false;
    }

    @Override
    public Boolean getGroundAlgae() {
        return false;
    }

    @Override
    public Boolean getScoreAlgae() {
        return false;
    }

    @Override
    public Boolean getCoralIntakeScoreL1() {
        return false;
    }

    @Override
    public LinearVelocity getManualElevatorVelocity() {
        return MetersPerSecond.of(0);
        // double axis = SimpleMath.povToVector(joystick.getPOV()).getY();
        // return Centimeters.of(50).per(Seconds).times(axis);
    }

    @Override
    public AngularVelocity getManualElevatorArmVelocity() {
        return RotationsPerSecond.of(0);
        // double axis = SimpleMath.povToVector(joystick.getPOV()).getY();
        // return Degrees.of(180).per(Seconds).times(axis);
    }

    @Override
    public Boolean getClimb() {
        return xbox_controller.getLeftTriggerAxis() > 0.3;
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
}
