package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.ElevatorIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Elevator extends KillableSubsystem implements ShuffleboardPublisher, PoweredSubsystem {
  private final ElevatorIO io;

  // Maximum elevator velocity and acceleration constraints
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // The plant holds a state-space model of our elevator. This system has the following properties:
  //
  // States: [position, velocity], in, meters and meters per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in meters.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N2, N1, N2> elevatorSystem =
      LinearSystemId.identifyPositionSystem(Constants.Elevator.kV, Constants.Elevator.kA);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N2> observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          elevatorSystem,
          VecBuilder.fill(
              Constants.Elevator.STD_STATE_POSITION,
              Constants.Elevator
                  .STD_STATE_VELOCITY), // Standard deviation of the state (position, velocity)
          VecBuilder.fill(
              Constants.Elevator.STD_ENCODER_POSITION,
              Constants.Elevator
                  .STD_ENCODER_VELOCITY), // Standard deviation of encoder measurements (position,
          // velocity)
          Constants.Elevator.kDt);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N2> controller =
      new LinearQuadraticRegulator<>(
          elevatorSystem,
          VecBuilder.fill(
              Constants.Elevator.REGULATOR_POSITION_ERROR_TOLERANCE,
              Constants.Elevator
                  .REGULATOR_VELOCITY_ERROR_TOLERANCE), // qelms. Positon, Velocity error tolerance,
          // in meters and meters per second.
          // Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(
              Constants.Elevator
                  .REGULATOR_CONTROL_EFFORT_TOLERANCE), // relms. Control effort (voltage)
          // tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          Constants.Elevator.kDt); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N2> loop =
      new LinearSystemLoop<>(elevatorSystem, controller, observer, 12.0, Constants.Elevator.kDt);

  public Elevator(ElevatorIO io) {
    this.io = io;

    io.applyTalonFXConfig(
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.Elevator.SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLowerLimit(Constants.Elevator.SUPPLY_CURRENT_LOWER_LIMIT)
                    .withSupplyCurrentLowerTime(1)
                    .withStatorCurrentLimit(Constants.Elevator.STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.setLeftMotorPosition(
        Constants.Elevator.STARTING_HEIGHT / Constants.Elevator.METERS_PER_ROTATION);
    io.setRightMotorPosition(
        Constants.Elevator.STARTING_HEIGHT / Constants.Elevator.METERS_PER_ROTATION);

    m_setpoint = new TrapezoidProfile.State(getCurrentHeight(), 0);
    set(Constants.Elevator.STARTING_HEIGHT);

    sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                Volts.of(4.5).per(Second),
                Volts.of(3.0),
                Seconds.of(1.2),
                (state -> Logger.recordOutput("Elevator/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(this::setBothMotors, null, this));

    controller.latencyCompensate(elevatorSystem, Constants.Elevator.kDt, 0.02022);

    SmartDashboard.putNumber("Elevator", Constants.Elevator.STARTING_HEIGHT);
  }

  private final SysIdRoutine sysIdRoutine;

  private double getCurrentRotationalVelocity() {
    return (io.getLeftMotorVelocity() + io.getRightMotorVelocity()) / 2;
  }

  private double getCurrentRotation() {
    return (io.getLeftMotorPosition() + io.getRightMotorPosition()) / 2;
  }

  /** Height of the elevator in meters */
  @AutoLogLevel(level = Level.Sysid)
  public double getCurrentHeight() {
    return getCurrentRotation() * Constants.Elevator.METERS_PER_ROTATION;
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getCurrentVelocity() {
    return getCurrentRotationalVelocity() * Constants.Elevator.METERS_PER_ROTATION;
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getCurrentVoltage() {
    return (io.getLeftMotorVoltage() + io.getRightMotorVoltage()) / 2;
  }

  @AutoLogLevel(level = Level.DebugReal)
  private boolean getBottomEndStopPressed() {
    return io.getBottomEndStop();
  }

  @AutoLogLevel(level = Level.DebugReal)
  private boolean getTopEndStopPressed() {
    return io.getTopEndStop();
  }

  public void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  @Override
  public void periodic() {

    // set(SmartDashboard.getNumber("Elevator", Constants.Elevator.STARTING_HEIGHT));

    // Get next setpoint from profile.
    m_setpoint = m_profile.calculate(Constants.Elevator.kDt, m_setpoint, m_goal);

    Logger.recordOutput("Elevator/setpoint", m_setpoint.velocity);

    // Set setpoint of the linear system (position m, velocity m/s).
    loop.setNextR(VecBuilder.fill(m_setpoint.position, m_setpoint.velocity));

    // Correct our Kalman filter's state vector estimate with encoder data.
    loop.correct(VecBuilder.fill(getCurrentHeight(), getCurrentVelocity()));
    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    loop.predict(Constants.Elevator.kDt);

    double nextVoltage =
        loop.getU(0)
            + Constants.Elevator.kG
            + Constants.Elevator.kS * Math.signum(m_setpoint.velocity);

    if (Math.abs(getCurrentVelocity()) < 0.26
        || ((!getTopEndStopPressed() || nextVoltage <= 0)
            && (!getBottomEndStopPressed() || nextVoltage >= 0))) {
      setBothMotors(nextVoltage);
    } else {
      setBothMotors(0);
    }

    // Update mechanism
    RobotContainer.model.elevator.update(getCurrentHeight());
    RobotContainer.model.elevator.updateSetpoint(m_setpoint.position);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  private void setBothMotors(double voltage) {
    io.setLeftMotorVoltage(voltage);
    io.setRightMotorVoltage(voltage);
  }

  private void setBothMotors(Voltage voltage) {
    setBothMotors(voltage.in(Volts));
  }

  public void set(double heightMeters) {
    m_goal = new TrapezoidProfile.State(heightMeters, 0.0);
  }

  public void moveTo(ElevatorHeight height) {
    set(height.getHeight());
  }

  @AutoLogLevel(level = Level.Real)
  public ElevatorHeight getNearestHeight() {
    double currentHeight = getCurrentHeight();
    double currentArmAngle = RobotContainer.elevatorArm.getArmAngle();

    var heights = ElevatorHeight.values();
    Arrays.sort(
        heights,
        (a, b) ->
            Double.compare(
                a.getDifference(currentHeight, currentArmAngle),
                b.getDifference(currentHeight, currentArmAngle)));
    return heights[0];
  }

  public boolean atGoal() {
    return loop.getError(0) < Constants.Elevator.AT_GOAL_POSITION_TOLERANCE
        && loop.getError(1) < Constants.Elevator.AT_GOAL_VELOCITY_TOLERANCE
        && m_goal.equals(m_setpoint);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void kill() {
    io.setLeftMotorVoltage(0);
    io.setRightMotorVoltage(0);
  }

  @Override
  public void close() throws Exception {
    io.close();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Elevator Target", m_goal.position, 0, ElevatorHeight.L4.getHeight())
        .subscribe(this::set);
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getLeftMotorCurrentDraw() + io.getRightMotorCurrentDraw();
  }
}
