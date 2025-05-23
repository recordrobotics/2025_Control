// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.ElevatorHeadIO;
import frc.robot.subsystems.io.sim.ElevatorHeadSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SysIdManager;
import org.littletonrobotics.junction.Logger;

public class ElevatorHead extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final ElevatorHeadIO io;

  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.ElevatorHead.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final PIDController pid =
      new PIDController(
          Constants.ElevatorHead.kP, Constants.ElevatorHead.kI, Constants.ElevatorHead.kD);
  private final ProfiledPIDController positionPid =
      new ProfiledPIDController(
          Constants.ElevatorHead.kP_position,
          Constants.ElevatorHead.kI_position,
          Constants.ElevatorHead.kD_position,
          new TrapezoidProfile.Constraints(
              Constants.ElevatorHead.POSITION_MODE_MAX_VELOCITY,
              Constants.ElevatorHead.POSITION_MODE_MAX_ACCELERATION));
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          Constants.ElevatorHead.kS, Constants.ElevatorHead.kV, Constants.ElevatorHead.kA);

  private CoralShooterStates currentState = CoralShooterStates.OFF;

  private double positionCached = 0;
  private double velocityCached = 0;
  private double voltageCached = 0;

  private boolean hasAlgae = false;
  private boolean waitingForAlgae = false;
  private boolean waitingForIntakeSpeed = false;

  public ElevatorHead(ElevatorHeadIO io) {
    this.io = io;

    set(CoralShooterStates.OFF); // initialize as off
    DashboardUI.Test.addSlider("Elevator Head", io.getPercent(), -1, 1).subscribe(io::setPercent);

    io.setPosition(0);
    positionCached = 0;
    positionPid.reset(0);

    positionPid.setTolerance(
        Constants.ElevatorHead.AT_GOAL_POSITION_TOLERANCE,
        Constants.ElevatorHead.AT_GOAL_VELOCITY_TOLERANCE);

    sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("ElevatorHead/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putNumber("ElevatorHead_Value", 0);
  }

  public ElevatorHeadSim getSimIO() throws Exception {
    if (io instanceof ElevatorHeadSim) {
      return (ElevatorHeadSim) io;
    } else {
      throw new Exception("ElevatorHeadIO is not a simulation");
    }
  }

  private final SysIdRoutine sysIdRoutine;

  public enum AlgaeGrabberStates {
    OUT_GROUND,
    OUT_REEF,
    SHOOT_BARGE,
    INTAKE_GROUND,
    INTAKE_REEF,
    HOLD_REEF,
    HOLD_GROUND,
    OFF;
  }

  public enum CoralShooterStates {
    OUT_FORWARD,
    OUT_BACKWARD,
    INTAKE,
    POSITION,
    OFF;
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getVelocity() {
    return velocityCached
        / 60.0
        / Constants.ElevatorHead.GEAR_RATIO
        * Math.PI
        * Constants.ElevatorHead.WHEEL_DIAMETER.in(Meters); /* RPM -> RPS */
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getPosition() {
    return positionCached
        / Constants.ElevatorHead.GEAR_RATIO
        * Math.PI
        * Constants.ElevatorHead.WHEEL_DIAMETER.in(Meters); /* Rotations -> Meters */
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getVoltage() {
    return voltageCached;
  }

  public void moveBy(Distance distance) {
    moveBy(distance.in(Meters));
  }

  public void moveBy(double meters) {
    set(0);
    currentState = CoralShooterStates.POSITION;
    positionPid.reset(getPosition());
    positionPid.setGoal(getPosition() + meters);
  }

  /** Set shooter speed */
  public void set(double speed) {
    pid.setSetpoint(speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void set(CoralShooterStates state) {
    waitingForAlgae = false;
    waitingForIntakeSpeed = false;
    hasAlgae = false;
    switch (state) {
      case OUT_FORWARD:
        currentState = CoralShooterStates.OUT_FORWARD;
        set(Constants.ElevatorHead.OUT_SPEED_FORWARD);
        break;
      case OUT_BACKWARD:
        currentState = CoralShooterStates.OUT_BACKWARD;
        set(Constants.ElevatorHead.OUT_SPEED_BACKWARD);
        break;
      case INTAKE:
        currentState = CoralShooterStates.INTAKE;
        set(Constants.ElevatorHead.INTAKE_SPEED);
        break;
      case POSITION:
        currentState = CoralShooterStates.POSITION;
        set(0);
        positionPid.setGoal(getPosition());
        break;
      case OFF: // Off
      default: // should never happen
        currentState = CoralShooterStates.OFF;
        set(0);
        break;
    }
  }

  public void set(AlgaeGrabberStates state) {
    currentState = CoralShooterStates.OFF;
    switch (state) {
      case OUT_GROUND:
        set(Constants.ElevatorHead.OUT_GROUND_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = false;
        break;
      case OUT_REEF:
        set(Constants.ElevatorHead.OUT_REEF_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = false;
        break;
      case SHOOT_BARGE:
        set(Constants.ElevatorHead.SHOOT_BARGE_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = false;
        break;
      case INTAKE_GROUND:
        set(Constants.ElevatorHead.INTAKE_GROUND_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = true;
        hasAlgae = false;
        break;
      case INTAKE_REEF:
        set(Constants.ElevatorHead.INTAKE_REEF_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = true;
        hasAlgae = false;
        break;
      case HOLD_REEF:
        set(Constants.ElevatorHead.HOLD_REEF_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = true;
        break;
      case HOLD_GROUND:
        set(Constants.ElevatorHead.HOLD_GROUND_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = true;
        break;
      case OFF: // Off
      default: // should never happen
        waitingForIntakeSpeed = false;
        waitingForAlgae = false;
        set(0);
        break;
    }
  }

  @AutoLogLevel(level = Level.Real)
  public boolean hasAlgae() {
    return hasAlgae;
  }

  @AutoLogLevel(level = Level.Real)
  public CoralShooterStates getCurrentCoralShooterState() {
    return currentState;
  }

  @AutoLogLevel(level = Level.Real)
  public boolean hasCoral() {
    return debounced_value;
  }

  public boolean positionAtGoal() {
    return positionPid.atGoal();
  }

  public boolean coralReady() {
    return Math.abs(getVelocity()) < 0.1 || (hasCoral() && positionAtGoal());
  }

  private double lastSpeed = 0;

  @Override
  public void periodicManaged() {

    positionCached = io.getPosition();
    velocityCached = io.getVelocity();
    if (Constants.RobotState.AUTO_LOG_LEVEL.isAtLeast(Level.Sysid)) {
      voltageCached = io.getVoltage();
    }

    // set(0);
    // currentState = CoralShooterStates.POSITION;
    // positionPid.setGoal(SmartDashboard.getNumber("CoralShooter_Value", 0));

    if (currentState == CoralShooterStates.POSITION) {
      double pidOutput = positionPid.calculate(getPosition());

      Logger.recordOutput("ElevatorHead/PositionSetpoint", positionPid.getSetpoint().position);

      double feedforwardOutput =
          feedForward.calculateWithVelocities(lastSpeed, positionPid.getSetpoint().velocity);

      if (SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.ElevatorHead) {
        io.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
      }

      lastSpeed = positionPid.getSetpoint().velocity;
    } else {
      double pidOutput = pid.calculate(getVelocity());
      double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());

      if (SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.ElevatorHead) {
        io.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
      }

      lastSpeed = pid.getSetpoint();

      if (waitingForIntakeSpeed && Math.abs(getVelocity()) > 1.0) { // TODO: tune
        waitingForIntakeSpeed = false;
        waitingForAlgae = true;
      }

      if (waitingForAlgae && Math.abs(getVelocity()) < 0.5) {
        hasAlgae = true;
        waitingForAlgae = false;
      }
    }

    debounced_value = !m_debouncer.calculate(io.getCoralDetector());
  }

  @Override
  public void simulationPeriodicManaged() {
    io.simulationPeriodic();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Elevator Head", io.getPercent(), -1, 1).subscribe(io::setPercent);
  }

  @Override
  public void kill() {
    set(CoralShooterStates.OFF);
    set(AlgaeGrabberStates.OFF);
    io.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getCurrentDrawAmps();
  }
}
