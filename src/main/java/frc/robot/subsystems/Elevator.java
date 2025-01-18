package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.Logger;

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

    sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(
                this::setBothMotors,
                // Tell SysId how to record a frame of data
                log -> {
                  log.motor("elevator")
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              motorLeft.get() * RobotController.getBatteryVoltage(), Volts))
                      .angularPosition(m_angle.mut_replace(getCurrentRotation(), Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              getCurrentRotationalVelocity(), RotationsPerSecond));
                },
                this));
  }

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Radians.mutable(0);
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  private final SysIdRoutine sysIdRoutine;

  private double getCurrentRotationalVelocity() {
    return (motorLeft.getVelocity().getValueAsDouble()
            + motorRight.getVelocity().getValueAsDouble())
        / 2;
  }

  private double getCurrentRotation() {
    return (motorLeft.getPosition().getValueAsDouble()
            + motorRight.getPosition().getValueAsDouble())
        / 2;
  }

  /** Average of the left and right heights of the elevator */
  private double getCurrentHeight() {
    return getCurrentRotation() / Constants.Elevator.METERS_PER_ROTATION;
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
      setBothMotors(fwVal + pidVal);
    }

    currentSetpoint = controller.getSetpoint();

    // Update mechanism
    RobotContainer.model.elevator.update(getCurrentHeight());
    RobotContainer.model.elevator.updateSetpoint(currentSetpoint.position);
  }

  private void setBothMotors(double voltage) {
    motorLeft.setVoltage(voltage);
    motorRight.setVoltage(voltage);
  }

  private void setBothMotors(Voltage voltage) {
    setBothMotors(voltage.in(Volts));
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
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
