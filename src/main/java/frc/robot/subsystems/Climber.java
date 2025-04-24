package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends KillableSubsystem implements ShuffleboardPublisher, PoweredSubsystem {

  private final ClimberIO io;
  private final SysIdRoutine sysIdRoutine;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          Constants.Climber.kP,
          Constants.Climber.kI,
          Constants.Climber.kD,
          new TrapezoidProfile.Constraints(
              Constants.Climber.MAX_ARM_VELOCITY, Constants.Climber.MAX_ARM_ACCELERATION));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.Climber.kS, Constants.Climber.kG, Constants.Climber.kV, Constants.Climber.kA);

  private ClimberState currentState = ClimberState.Park;

  private double climbStartTime = 0;

  public Climber(ClimberIO io) {
    this.io = io;

    io.applyTalonFXConfig(
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.setPosition(
        Constants.Climber.START_ANGLE.in(Radians) / Constants.Climber.ARM_RADIANS_PER_ROTATION);
    set(ClimberState.Park);

    pid.setTolerance(0.15, 1.05);

    pid.reset(Constants.Climber.START_ANGLE.in(Radians));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second),
                Volts.of(1.5),
                Seconds.of(1.3),
                (state -> Logger.recordOutput("Climber/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putNumber("Climber", Constants.Climber.START_ANGLE.in(Radians));
  }

  public enum ClimberState {
    Park,
    Extend,
    Climb
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  @Override
  public void periodic() {
    pid.setGoal(SmartDashboard.getNumber("Climber", Constants.Climber.START_ANGLE.in(Radians)));

    if (currentState == ClimberState.Climb) {
      double rampT =
          MathUtil.clamp(
              (Timer.getTimestamp() - climbStartTime)
                  / Constants.Climber.CLIMB_RAMP_TIME.in(Seconds),
              0,
              1);
      double rampedVoltage =
          MathUtil.interpolate(
              Constants.Climber.CLIMB_RAMP_VOLTAGE_START.in(Volts),
              Constants.Climber.CLIMB_RAMP_VOLTAGE_END.in(Volts),
              rampT);
      if (atGoal()) {
        io.setVoltage(Constants.Climber.CLIMB_HOLD_VOLTAGE.in(Volts));
      } else {
        io.setVoltage(rampedVoltage);
      }
    } else {
      double pidOutputArm = pid.calculate(getArmAngle());

      double feedforwardOutput =
          feedforward.calculateWithVelocities(
              getArmAngle(), currentSetpoint.velocity, pid.getSetpoint().velocity);

      Logger.recordOutput("Climber/PID", pidOutputArm);
      Logger.recordOutput("Climber/FF", feedforwardOutput);
      Logger.recordOutput("Climber/Setpoint", currentSetpoint.position);

      io.setVoltage(pidOutputArm + feedforwardOutput);
      currentSetpoint = pid.getSetpoint();
    }

    // Update mechanism
    RobotContainer.model.climber.update(getArmAngle());
    RobotContainer.model.climber.updateSetpoint(currentSetpoint.position);
  }

  public boolean atGoal() {
    return pid.atGoal();
  }

  public void set(ClimberState state) {
    currentState = state;
    switch (state) {
      case Park:
        pid.setGoal(Constants.Climber.PARK_ANGLE.in(Radians));
        break;
      case Extend:
        pid.setGoal(Constants.Climber.EXTENDED_ANGLE.in(Radians));
        break;
      case Climb:
        climbStartTime = Timer.getTimestamp();
        pid.setGoal(Constants.Climber.CLIMBED_ANGLE.in(Radians));
        break;
    }
  }

  @AutoLogOutput
  public ClimberState getCurrentState() {
    return currentState;
  }

  @AutoLogOutput
  public double getArmAngle() {
    return io.getPosition() * Constants.Climber.ARM_RADIANS_PER_ROTATION;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return io.getVelocity() * Constants.Climber.ARM_RADIANS_PER_ROTATION;
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getVoltage();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public ClimberSim getSimIO() throws Exception {
    if (io instanceof ClimberSim) {
      return (ClimberSim) io;
    } else {
      throw new Exception("ClimberIO is not a simulation");
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {}

  @Override
  public void kill() {
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
