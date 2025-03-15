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
import frc.robot.subsystems.io.CoralShooterIO;
import frc.robot.subsystems.io.sim.CoralShooterSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralShooter extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final CoralShooterIO io;

  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralShooter.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final PIDController pid =
      new PIDController(
          Constants.CoralShooter.kP, Constants.CoralShooter.kI, Constants.CoralShooter.kD);
  private final ProfiledPIDController positionPid =
      new ProfiledPIDController(
          Constants.CoralShooter.kP_position,
          Constants.CoralShooter.kI_position,
          Constants.CoralShooter.kD_position,
          new TrapezoidProfile.Constraints(
              Constants.CoralShooter.POSITION_MODE_MAX_VELOCITY,
              Constants.CoralShooter.POSITION_MODE_MAX_ACCELERATION));
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          Constants.CoralShooter.kS, Constants.CoralShooter.kV, Constants.CoralShooter.kA);

  private CoralShooterStates currentState = CoralShooterStates.OFF;

  public CoralShooter(CoralShooterIO io) {
    this.io = io;

    toggle(CoralShooterStates.OFF); // initialize as off
    DashboardUI.Test.addSlider("Coral Shooter", io.getPercent(), -1, 1).subscribe(io::setPercent);

    io.setPosition(0);
    positionPid.reset(0);

    positionPid.setTolerance(
        Constants.CoralShooter.AT_GOAL_POSITION_TOLERANCE,
        Constants.CoralShooter.AT_GOAL_VELOCITY_TOLERANCE);

    sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("CoralShooter/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putNumber("CoralShooter_Value", 0);
  }

  public CoralShooterSim getSimIO() throws Exception {
    if (io instanceof CoralShooterSim) {
      return (CoralShooterSim) io;
    } else {
      throw new Exception("CoralShooterIO is not a simulation");
    }
  }

  private final SysIdRoutine sysIdRoutine;

  public enum CoralShooterStates {
    OUT_FORWARD,
    OUT_BACKWARD,
    INTAKE,
    POSITION,
    OFF;
  }

  @AutoLogOutput
  public double getVelocity() {
    return io.getVelocity()
        / 60.0
        / Constants.CoralShooter.GEAR_RATIO
        * Math.PI
        * Constants.CoralShooter.WHEEL_DIAMETER.in(Meters); /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getPosition() {
    return io.getPosition()
        / Constants.CoralShooter.GEAR_RATIO
        * Math.PI
        * Constants.CoralShooter.WHEEL_DIAMETER.in(Meters); /* Rotations -> Meters */
  }

  @AutoLogOutput
  public double getVoltage() {
    return io.getVoltage();
  }

  public void moveBy(Distance distance) {
    moveBy(distance.in(Meters));
  }

  public void moveBy(double meters) {
    toggle(0);
    currentState = CoralShooterStates.POSITION;
    positionPid.reset(getPosition());
    positionPid.setGoal(getPosition() + meters);
  }

  /** Set shooter speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralShooterStates state) {
    switch (state) {
      case OUT_FORWARD:
        currentState = CoralShooterStates.OUT_FORWARD;
        toggle(Constants.CoralShooter.OUT_SPEED_FORWARD);
        break;
      case OUT_BACKWARD:
        currentState = CoralShooterStates.OUT_BACKWARD;
        toggle(Constants.CoralShooter.OUT_SPEED_BACKWARD);
        break;
      case INTAKE:
        currentState = CoralShooterStates.INTAKE;
        toggle(Constants.CoralShooter.INTAKE_SPEED);
        break;
      case POSITION:
        currentState = CoralShooterStates.POSITION;
        toggle(0);
        positionPid.setGoal(getPosition());
        break;
      case OFF: // Off
      default: // should never happen
        currentState = CoralShooterStates.OFF;
        toggle(0);
        break;
    }
  }

  @AutoLogOutput
  public CoralShooterStates getCurrentState() {
    return currentState;
  }

  @AutoLogOutput
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
  public void periodic() {
    // toggle(0);
    // currentState = CoralShooterStates.POSITION;
    // positionPid.setGoal(SmartDashboard.getNumber("CoralShooter_Value", 0));

    if (currentState == CoralShooterStates.POSITION) {
      double pidOutput = positionPid.calculate(getPosition());

      Logger.recordOutput("CoralShooter/PositionSetpoint", positionPid.getSetpoint().position);

      double feedforwardOutput =
          feedForward.calculateWithVelocities(lastSpeed, positionPid.getSetpoint().velocity);

      io.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
      lastSpeed = positionPid.getSetpoint().velocity;
    } else {
      double pidOutput = pid.calculate(getVelocity());
      double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
      io.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
      lastSpeed = pid.getSetpoint();
    }

    debounced_value = !m_debouncer.calculate(io.getCoralDetector());
  }

  @Override
  public void simulationPeriodic() {
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
    DashboardUI.Test.addSlider("Coral Shooter", io.getPercent(), -1, 1).subscribe(io::setPercent);
  }

  @Override
  public void kill() {
    toggle(CoralShooterStates.OFF);
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
