package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;

public class Elevator extends KillableSubsystem implements ShuffleboardPublisher {

  private final TalonFX motorLeft;
  private final TalonFX motorRight;
  private final DigitalInput bottomEndStop;
  private final DigitalInput topEndStop;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.Elevator.kP,
          Constants.Elevator.kI,
          Constants.Elevator.kD,
          constraints,
          Constants.Elevator.kDt);
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          Constants.Elevator.kS,
          Constants.Elevator.kG,
          Constants.Elevator.kV,
          Constants.Elevator.kA);

  public Elevator() {
    motorLeft = new TalonFX(RobotMap.Elevator.MOTOR_LEFT_ID);
    motorRight = new TalonFX(RobotMap.Elevator.MOTOR_RIGHT_ID);
    bottomEndStop = new DigitalInput(RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
    topEndStop = new DigitalInput(RobotMap.Elevator.TOP_ENDSTOP_ID);

    controller.setTolerance(
        Constants.Elevator.AT_GOAL_POSITION_TOLERANCE,
        Constants.Elevator.AT_GOAL_VELOCITY_TOLERANCE);

    motorLeft.setPosition(0);
    motorRight.setPosition(0);
  }

  private double getCurrentHeightLeft() {
    return motorLeft.getPosition().getValueAsDouble() / Constants.Elevator.METERS_PER_ROTATION;
  }

  private double getCurrentHeightRight() {
    return motorRight.getPosition().getValueAsDouble() / Constants.Elevator.METERS_PER_ROTATION;
  }

  /** Average of the left and right heights of the elevator */
  private double getCurrentHeight() {
    return (getCurrentHeightLeft() + getCurrentHeightRight()) / 2;
  }

  private boolean getBottomEndStopPressed() {
    return bottomEndStop.get();
  }

  private boolean getTopEndStopPressed() {
    return topEndStop.get();
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  @Override
  public void periodic() {
    // Run controller and update motor output
    double pidVal = controller.calculate(getCurrentHeight());
    double fwVal =
        feedforward.calculateWithVelocities(
            currentSetpoint.velocity, controller.getSetpoint().velocity);

    if ((!getTopEndStopPressed() || pidVal <= 0) && (!getBottomEndStopPressed() || pidVal >= 0)) {
      motorLeft.setVoltage(pidVal + fwVal);
      motorRight.setVoltage(pidVal + fwVal);
    }

    currentSetpoint = controller.getSetpoint();
  }

  public void toggle(double heightMeters) {
    controller.setGoal(heightMeters);
  }

  public void moveTo(ElevatorHeight height) {
    switch (height) {
      case OFF:
        kill();
        break;
      default:
        toggle(height.getHeight());
        break;
    }
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  @Override
  public void kill() {
    motorLeft.setVoltage(0);
    motorRight.setVoltage(0);
  }

  @Override
  public void close() {
    motorLeft.close();
    motorRight.close();
    bottomEndStop.close();
    topEndStop.close();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider(
            "Elevator Target", controller.getGoal().position, 0, ElevatorHeight.L4.getHeight())
        .subscribe(this::toggle);
  }
}
